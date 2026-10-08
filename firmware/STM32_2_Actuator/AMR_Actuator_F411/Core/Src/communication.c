#include "communication.h"

#include "mechanism.h"
#include "mechanism_config.h"
#include "usbd_cdc_if.h"

#include <string.h>
#include <stdio.h>


/* =========================================================
 * COMMAND VARIABLE IN main.c
 * ========================================================= */

extern volatile uint8_t g_mech_cmd;


/* =========================================================
 * USB RX LINE QUEUE - V2.2
 *
 * V1 used a single s_rx_ready flag.  Once one newline was
 * received, every remaining byte in the same USB packet was
 * discarded until the main loop consumed the command.
 *
 * V2 stores complete command lines in a small ring queue.
 * The USB callback only writes s_rx_head; the main loop only
 * writes s_rx_tail, avoiding a shared read/modify/write count.
 * ========================================================= */

#define COMM_RX_SIZE          32U
#define COMM_RX_QUEUE_COUNT    8U

static char s_rx_build[COMM_RX_SIZE];
static volatile uint8_t s_rx_build_len = 0U;
static volatile uint8_t s_rx_discard_line = 0U;

static char s_rx_queue[COMM_RX_QUEUE_COUNT][COMM_RX_SIZE];
static volatile uint8_t s_rx_head = 0U;
static volatile uint8_t s_rx_tail = 0U;

static volatile uint8_t s_rx_overflow_event = 0U;
static volatile uint8_t s_rx_queue_full_event = 0U;

/* After CDC open/reconnect, force one last parser reset at the
 * first actual OUT packet. This protects against control-line
 * ordering differences between Linux hosts / CDC drivers. */
static volatile uint8_t s_rx_first_packet_after_session = 1U;

/* =========================================================
 * V2.8 CLOSE / OPEN-FULL / CLOSED-FULL GUARD
 *
 * Safety invariant:
 *   - After boot / reconnect / STOP / RESET / OPEN / recovery,
 *     closing is LOCKED.
 *   - ARM_CLOSE is accepted only from MECH_OPEN_DONE.
 *   - PICK or CLOSE consumes the arm immediately (single-use).
 *
 * This prevents a repeated CLOSE command from retracting the clamp
 * farther when the physical mechanism is already at its closed end.
 * ========================================================= */
static volatile uint8_t s_close_armed = 0U;

/* =========================================================
 * V2.5 OPEN-FULL GUARD
 *
 * Physical-position safety invariant:
 *   - OPEN_FULL=1 only after a timed full OPEN / full release, or
 *     an explicit operator CONFIRM_OPEN_FULL while stopped.
 *   - OPEN is rejected when OPEN_FULL=1, preventing the clamp from
 *     driving harder into the mechanical open end stop.
 *   - ARM_CLOSE requires OPEN_FULL=1. IDLE alone is never enough.
 *   - Any actual closing/pick motion consumes OPEN_FULL immediately.
 *   - USB reconnect / RESET clears manual position confidence.
 *
 * CONFIRM_OPEN_FULL never moves a motor. It exists to synchronize
 * software state with a physically verified full-open clamp after
 * manual commissioning / interrupted timed motion.
 * ========================================================= */
static volatile uint8_t s_open_full_confirmed = 0U;

/* V2.7: partial-close continuation state.
 * This is only a physical-position confidence flag; it never moves
 * the motor or arms CLOSE by itself. */
static volatile uint8_t s_close_resume_allowed = 0U;

/* V2.8: explicit CLOSED-FULL physical-position confidence.
 * This flag is set only after operator confirmation while stopped.
 * It blocks any further CLOSE/PICK/close-arm command until an OPEN
 * motion begins or the confirmation is explicitly cleared. */
static volatile uint8_t s_closed_full_confirmed = 0U;


/* =========================================================
 * USB TX QUEUE
 * ========================================================= */

#define COMM_TX_QUEUE_COUNT   12U
#define COMM_TX_SIZE         192U

static char s_tx_queue[COMM_TX_QUEUE_COUNT][COMM_TX_SIZE];

static uint8_t s_tx_head = 0U;
static uint8_t s_tx_tail = 0U;
static uint8_t s_tx_count = 0U;


/* =========================================================
 * STATE TRACKING
 * ========================================================= */

static uint8_t s_prev_state = 255U;
static uint8_t s_prev_fault = 255U;


/* =========================================================
 * INTERNAL TX
 * ========================================================= */

static void Comm_QueueTx(const char *text)
{
    if (text == NULL)
        return;

    if (s_tx_count >= COMM_TX_QUEUE_COUNT)
        return;

    strncpy(s_tx_queue[s_tx_head],
            text,
            COMM_TX_SIZE - 1U);

    s_tx_queue[s_tx_head][COMM_TX_SIZE - 1U] = '\0';

    s_tx_head++;

    if (s_tx_head >= COMM_TX_QUEUE_COUNT)
        s_tx_head = 0U;

    s_tx_count++;
}


static void Comm_TxTask(void)
{
    uint16_t len;

    if (s_tx_count == 0U)
        return;

    len = (uint16_t)strlen(s_tx_queue[s_tx_tail]);

    if (CDC_Transmit_FS(
            (uint8_t *)s_tx_queue[s_tx_tail],
            len) == USBD_OK)
    {
        s_tx_tail++;

        if (s_tx_tail >= COMM_TX_QUEUE_COUNT)
            s_tx_tail = 0U;

        s_tx_count--;
    }
}


/* =========================================================
 * INTERNAL RX QUEUE
 * ========================================================= */

static uint8_t Comm_PopRxLine(char *out)
{
    uint8_t tail;

    if (out == NULL)
        return 0U;

    tail = s_rx_tail;

    if (tail == s_rx_head)
        return 0U;

    memcpy(out,
           s_rx_queue[tail],
           COMM_RX_SIZE);

    tail++;

    if (tail >= COMM_RX_QUEUE_COUNT)
        tail = 0U;

    s_rx_tail = tail;

    return 1U;
}


/* =========================================================
 * STATUS
 * ========================================================= */

static void Comm_SendStatus(void)
{
    char msg[COMM_TX_SIZE];

    uint32_t fsr;
    uint8_t state;
    uint8_t fault;
    uint8_t busy;
    uint8_t recover_open;
    uint8_t close_armed;
    uint8_t open_full;
    uint8_t close_resume;
    uint8_t closed_full;
    uint8_t pos_valid;
    uint16_t pos_permille;
    uint32_t recover_ms;

    float distance;
    int32_t distance_x10;

    fsr = Mechanism_GetFSR();
    state = (uint8_t)Mechanism_GetState();
    fault = (uint8_t)Mechanism_GetFault();
    busy = Mechanism_IsBusy();
    recover_open = Mechanism_CanRecoverOpen();
    close_armed = s_close_armed;
    open_full = s_open_full_confirmed;
    close_resume = s_close_resume_allowed;
    closed_full = s_closed_full_confirmed;
    pos_valid = Mechanism_ClampPositionValid();
    pos_permille = Mechanism_GetClampPositionPermille();
    recover_ms = Mechanism_GetRecoverOpenPlannedMs();
    distance = Mechanism_GetDistance();

    if (distance < 0.0f)
    {
        distance_x10 = -1;
    }
    else
    {
        distance_x10 = (int32_t)(distance * 10.0f);
    }

    snprintf(msg,
             sizeof(msg),
             "STATUS STATE=%u FAULT=%u BUSY=%u RECOVER_OPEN=%u CLOSE_ARMED=%u OPEN_FULL=%u CLOSED_FULL=%u CLOSE_RESUME=%u POS_VALID=%u POS_PERMILLE=%u RECOVER_MS=%lu FSR=%lu DIST_X10=%ld\r\n",
             state, fault, busy, recover_open, close_armed, open_full, closed_full, close_resume,
             pos_valid, (unsigned int)pos_permille, (unsigned long)recover_ms,
             (unsigned long)fsr, (long)distance_x10);

    Comm_QueueTx(msg);
}


/* =========================================================
 * COMMAND PARSER
 * ========================================================= */

static void Comm_ProcessCommand(const char *cmd)
{
    char msg[COMM_TX_SIZE];

    uint8_t fault = (uint8_t)Mechanism_GetFault();
    uint8_t state = (uint8_t)Mechanism_GetState();

    if (cmd == NULL)
        return;


    /* -----------------------------------------------------
     * PING / VERSION / STATUS are always available.
     * ----------------------------------------------------- */

    if (strcmp(cmd, "PING") == 0)
    {
        Comm_QueueTx("PONG\r\n");
        return;
    }

    if (strcmp(cmd, "VERSION") == 0)
    {
        Comm_QueueTx("VERSION AMR_ACTUATOR_F411_V2_12_PICK_LIFT_FIX\r\n");
        return;
    }

    if (strcmp(cmd, "STATUS") == 0)
    {
        Comm_SendStatus();
        return;
    }

    /* -----------------------------------------------------
     * V2.10 MANUAL COMMISSIONING JOG
     * Fixed 300 ms pulses. These intentionally bypass the
     * OPEN_FULL/CLOSED_FULL/CLOSE_RESUME/POS_PERMILLE guards.
     * The position estimator is invalidated because manual jog
     * makes its previous estimate untrustworthy.
     * ----------------------------------------------------- */
    if (strcmp(cmd, "JOG_CLOSE") == 0)
    {
        s_close_armed = 0U;
        s_open_full_confirmed = 0U;
        s_closed_full_confirmed = 0U;
        s_close_resume_allowed = 0U;
        Mechanism_ClampPositionInvalidate();

        if (g_mech_cmd != 0U)
        {
            Comm_QueueTx("ERR CMD_PENDING\r\n");
            return;
        }

        if (Mechanism_StartJogClose(300U))
            Comm_QueueTx("ACK JOG_CLOSE MS=300\r\n");
        else
            Comm_QueueTx("ERR JOG_CLOSE FAULT_OR_BUSY\r\n");
        return;
    }

    if (strcmp(cmd, "JOG_OPEN") == 0)
    {
        s_close_armed = 0U;
        s_open_full_confirmed = 0U;
        s_closed_full_confirmed = 0U;
        s_close_resume_allowed = 0U;
        Mechanism_ClampPositionInvalidate();

        if (g_mech_cmd != 0U)
        {
            Comm_QueueTx("ERR CMD_PENDING\r\n");
            return;
        }

        if (Mechanism_StartJogOpen(300U))
            Comm_QueueTx("ACK JOG_OPEN MS=300\r\n");
        else
            Comm_QueueTx("ERR JOG_OPEN FAULT_OR_BUSY\r\n");
        return;
    }


    /*
     * TIME-BASED LIFT COMMANDS - NO HC-SR04
     *
     * Compatibility commands keep the names currently used by HT_V1_ACT_AMR.py.
     * Their actual behavior follows the current hardware requirement:
     *   JOG_LIFT_UP_1S   -> UP for 10 s
     *   JOG_LIFT_DOWN_1S -> DOWN full for 10 s
     *
     * Truthful aliases are also exposed for direct commissioning.
     */
    if ((strcmp(cmd, "JOG_LIFT_UP_1S") == 0) ||
        (strcmp(cmd, "LIFT_UP_10S") == 0))
    {
        if (g_mech_cmd != 0U)
        {
            Comm_QueueTx("ERR CMD_PENDING\r\n");
            return;
        }

        if (Mechanism_StartJogLiftUp(LIFT_UP_TIME_MS))
            Comm_QueueTx("ACK LIFT_UP_10S MS=10000\r\n");
        else
            Comm_QueueTx("ERR LIFT_UP_10S FAULT_OR_BUSY\r\n");
        return;
    }

    if ((strcmp(cmd, "JOG_LIFT_DOWN_1S") == 0) ||
        (strcmp(cmd, "LIFT_DOWN_FULL") == 0))
    {
        if (g_mech_cmd != 0U)
        {
            Comm_QueueTx("ERR CMD_PENDING\r\n");
            return;
        }

        if (Mechanism_StartJogLiftDown(LIFT_DOWN_FULL_TIME_MS))
            Comm_QueueTx("ACK LIFT_DOWN_FULL MS=10000\r\n");
        else
            Comm_QueueTx("ERR LIFT_DOWN_FULL FAULT_OR_BUSY\r\n");
        return;
    }


    /* -----------------------------------------------------
     * V2.3 TEST BUILD ONLY: inject a clamp-timeout FAULT
     * without moving any actuator.  Strictly IDLE-only.
     * ----------------------------------------------------- */

    if (strcmp(cmd, "TEST_FAULT_CLAMP") == 0)
    {
        /* Any fault test/recovery context is OPEN-only. */
        s_close_armed = 0U;
        s_open_full_confirmed = 0U;
        s_close_resume_allowed = 0U;

        if (g_mech_cmd != 0U)
        {
            Comm_QueueTx("ERR CMD_PENDING\r\n");
            return;
        }

        if (Mechanism_TestInjectClampFault())
        {
            Comm_QueueTx("ACK TEST_FAULT_CLAMP FAULT=2\r\n");
        }
        else
        {
            snprintf(msg,
                     sizeof(msg),
                     "ERR TEST_FAULT_NOT_ALLOWED STATE=%u FAULT=%u BUSY=%u\r\n",
                     (unsigned int)Mechanism_GetState(),
                     (unsigned int)Mechanism_GetFault(),
                     (unsigned int)Mechanism_IsBusy());
            Comm_QueueTx(msg);
        }
        return;
    }


    /* -----------------------------------------------------
     * STOP is always accepted and never clears a fault.
     * ----------------------------------------------------- */

    if (strcmp(cmd, "STOP") == 0)
    {
        s_close_armed = 0U;
        g_mech_cmd = 9U;
        Comm_QueueTx("ACK STOP\r\n");
        return;
    }


    /* -----------------------------------------------------
     * RESET / RESET_FAULT stop all motors and clear the
     * software fault latch.  They do not start any motion.
     * ----------------------------------------------------- */

    if (strcmp(cmd, "RESET") == 0)
    {
        s_close_armed = 0U;
        s_open_full_confirmed = 0U;
        s_closed_full_confirmed = 0U;
        s_close_resume_allowed = 0U;
        Mechanism_ClampPositionInvalidate();
        g_mech_cmd = 20U;
        Comm_QueueTx("ACK RESET\r\n");
        return;
    }

    if (strcmp(cmd, "RESET_FAULT") == 0)
    {
        s_close_armed = 0U;
        s_open_full_confirmed = 0U;
        s_closed_full_confirmed = 0U;
        s_close_resume_allowed = 0U;
        Mechanism_ClampPositionInvalidate();
        g_mech_cmd = 20U;
        Comm_QueueTx("ACK RESET_FAULT\r\n");
        return;
    }


    /* -----------------------------------------------------
     * Safe recovery OPEN is allowed only for clamp-related
     * faults.  It is intentionally parsed before the generic
     * FAULT gate below.
     * ----------------------------------------------------- */

    if (strcmp(cmd, "RECOVER_OPEN") == 0)
    {
        s_close_armed = 0U;
        s_open_full_confirmed = 0U;
        s_closed_full_confirmed = 0U;

        if (g_mech_cmd != 0U)
        {
            Comm_QueueTx("ERR CMD_PENDING\r\n");
            return;
        }

        if (!Mechanism_CanRecoverOpen())
        {
            snprintf(msg, sizeof(msg),
                     "ERR RECOVER_OPEN_NOT_ALLOWED STATE=%u FAULT=%u POS_VALID=%u\r\n",
                     state, fault, (unsigned int)Mechanism_ClampPositionValid());
            Comm_QueueTx(msg);
            return;
        }

        snprintf(msg, sizeof(msg),
                 "ACK RECOVER_OPEN POS_PERMILLE=%u TARGET_MS=%lu\r\n",
                 (unsigned int)Mechanism_GetClampPositionPermille(),
                 (unsigned long)Mechanism_GetRecoverOpenPlannedMs());
        g_mech_cmd = 21U;
        Comm_QueueTx(msg);
        return;
    }


    /* -----------------------------------------------------
     * V2.5 PHYSICAL OPEN-FULL CONFIRMATION
     *
     * These commands never energize a motor. CONFIRM_OPEN_FULL is
     * accepted only when the mechanism is stopped and fault-free.
     * The operator must physically verify the clamp is fully open.
     * ----------------------------------------------------- */

    if (strcmp(cmd, "CLEAR_OPEN_FULL") == 0)
    {
        s_open_full_confirmed = 0U;
        s_close_armed = 0U;
        s_close_resume_allowed = 0U;
        Mechanism_ClampPositionInvalidate();
        Comm_QueueTx("ACK CLEAR_OPEN_FULL OPEN_FULL=0 CLOSE_ARMED=0 CLOSE_RESUME=0\r\n");
        return;
    }

    if (strcmp(cmd, "CONFIRM_OPEN_FULL") == 0)
    {
        if (fault != 0U)
        {
            snprintf(msg, sizeof(msg), "ERR CONFIRM_OPEN_FULL_FAULT %u\r\n", fault);
            Comm_QueueTx(msg);
            return;
        }

        if (Mechanism_IsBusy() || (g_mech_cmd != 0U))
        {
            Comm_QueueTx("ERR CONFIRM_OPEN_FULL_BUSY\r\n");
            return;
        }

        if ((Mechanism_GetState() != MECH_IDLE) &&
            (Mechanism_GetState() != MECH_OPEN_DONE) &&
            (Mechanism_GetState() != MECH_DROP_DONE))
        {
            snprintf(msg,
                     sizeof(msg),
                     "ERR CONFIRM_OPEN_FULL_STATE %u\r\n",
                     (unsigned int)Mechanism_GetState());
            Comm_QueueTx(msg);
            return;
        }

        s_open_full_confirmed = 1U;
        s_closed_full_confirmed = 0U;
        s_close_armed = 0U;
        s_close_resume_allowed = 0U;
        Mechanism_ClampPositionSetOpenFull();
        Comm_QueueTx("ACK CONFIRM_OPEN_FULL OPEN_FULL=1 CLOSE_ARMED=0 CLOSE_RESUME=0\r\n");
        return;
    }


    /* -----------------------------------------------------
     * V2.8 PHYSICAL CLOSED-FULL CONFIRMATION
     *
     * These commands never energize a motor. CLOSED_FULL is
     * deliberately operator-confirmed for now because the timed
     * CLOSE duration is still being commissioned. Once confirmed,
     * all close/pick arming is blocked until OPEN starts or the
     * confirmation is explicitly cleared.
     * ----------------------------------------------------- */

    if (strcmp(cmd, "CLEAR_CLOSED_FULL") == 0)
    {
        s_closed_full_confirmed = 0U;
        s_close_armed = 0U;
        s_close_resume_allowed = 0U;
        Mechanism_ClampPositionInvalidate();
        Comm_QueueTx("ACK CLEAR_CLOSED_FULL CLOSED_FULL=0 CLOSE_ARMED=0 CLOSE_RESUME=0\r\n");
        return;
    }

    if (strcmp(cmd, "CONFIRM_CLOSED_FULL") == 0)
    {
        if (fault != 0U)
        {
            snprintf(msg, sizeof(msg), "ERR CONFIRM_CLOSED_FULL_FAULT %u\r\n", fault);
            Comm_QueueTx(msg);
            return;
        }

        if (Mechanism_IsBusy() || (g_mech_cmd != 0U))
        {
            Comm_QueueTx("ERR CONFIRM_CLOSED_FULL_BUSY\r\n");
            return;
        }

        if ((Mechanism_GetState() != MECH_IDLE) &&
            (Mechanism_GetState() != MECH_CLOSE_DONE))
        {
            snprintf(msg,
                     sizeof(msg),
                     "ERR CONFIRM_CLOSED_FULL_STATE %u\r\n",
                     (unsigned int)Mechanism_GetState());
            Comm_QueueTx(msg);
            return;
        }

        s_closed_full_confirmed = 1U;
        s_open_full_confirmed = 0U;
        s_close_armed = 0U;
        s_close_resume_allowed = 0U;
        Mechanism_ClampPositionSetClosedFull();
        Comm_QueueTx("ACK CONFIRM_CLOSED_FULL CLOSED_FULL=1 OPEN_FULL=0 CLOSE_ARMED=0 CLOSE_RESUME=0\r\n");
        return;
    }


    /* -----------------------------------------------------
     * V2.5 CLOSE GUARD CONTROL
     *
     * DISARM_CLOSE is always safe. ARM_CLOSE is intentionally
     * strict: the mechanism must have completed a full OPEN and
     * be in MECH_OPEN_DONE. IDLE is NOT enough because IDLE does
     * not prove the physical clamp position.
     * ----------------------------------------------------- */

    if (strcmp(cmd, "CONFIRM_PARTIAL_CLOSE") == 0)
    {
        if (fault != 0U)
        {
            snprintf(msg, sizeof(msg), "ERR CONFIRM_PARTIAL_CLOSE_FAULT %u\r\n", fault);
            Comm_QueueTx(msg);
            return;
        }

        if (Mechanism_IsBusy() || (g_mech_cmd != 0U))
        {
            Comm_QueueTx("ERR CONFIRM_PARTIAL_CLOSE_BUSY\r\n");
            return;
        }

        if (s_open_full_confirmed)
        {
            Comm_QueueTx("ERR CONFIRM_PARTIAL_CLOSE_OPEN_FULL=1\r\n");
            return;
        }

        if (s_closed_full_confirmed)
        {
            Comm_QueueTx("ERR CONFIRM_PARTIAL_CLOSE_CLOSED_FULL=1\r\n");
            return;
        }

        s_close_armed = 0U;
        s_close_resume_allowed = 1U;
        Comm_QueueTx("ACK CONFIRM_PARTIAL_CLOSE CLOSE_RESUME=1 CLOSE_ARMED=0\r\n");
        return;
    }

    if (strcmp(cmd, "ARM_CLOSE_CONTINUE") == 0)
    {
        if (fault != 0U)
        {
            snprintf(msg, sizeof(msg), "ERR ARM_CLOSE_CONTINUE_FAULT %u\r\n", fault);
            Comm_QueueTx(msg);
            return;
        }

        if (Mechanism_IsBusy() || (g_mech_cmd != 0U))
        {
            Comm_QueueTx("ERR ARM_CLOSE_CONTINUE_BUSY\r\n");
            return;
        }

        if (s_closed_full_confirmed)
        {
            Comm_QueueTx("ERR CLOSE_GUARD_ALREADY_CLOSED_FULL CMD=ARM_CLOSE_CONTINUE CLOSED_FULL=1\r\n");
            return;
        }

        if (s_open_full_confirmed || !s_close_resume_allowed)
        {
            snprintf(msg, sizeof(msg),
                     "ERR ARM_CLOSE_CONTINUE_NOT_ALLOWED OPEN_FULL=%u CLOSE_RESUME=%u\r\n",
                     (unsigned int)s_open_full_confirmed,
                     (unsigned int)s_close_resume_allowed);
            Comm_QueueTx(msg);
            return;
        }

        s_close_armed = 1U;
        Comm_QueueTx("ACK ARM_CLOSE_CONTINUE CLOSE_ARMED=1 CLOSE_RESUME=1\r\n");
        return;
    }

    if (strcmp(cmd, "DISARM_CLOSE") == 0)
    {
        s_close_armed = 0U;
        Comm_QueueTx("ACK DISARM_CLOSE\r\n");
        return;
    }

    if (strcmp(cmd, "ARM_CLOSE") == 0)
    {
        if (fault != 0U)
        {
            snprintf(msg, sizeof(msg), "ERR ARM_CLOSE_FAULT %u\r\n", fault);
            Comm_QueueTx(msg);
            return;
        }

        if (Mechanism_IsBusy() || (g_mech_cmd != 0U))
        {
            Comm_QueueTx("ERR ARM_CLOSE_BUSY\r\n");
            return;
        }

        if (s_closed_full_confirmed)
        {
            Comm_QueueTx("ERR CLOSE_GUARD_ALREADY_CLOSED_FULL CMD=ARM_CLOSE CLOSED_FULL=1\r\n");
            return;
        }

        if (!s_open_full_confirmed)
        {
            snprintf(msg,
                     sizeof(msg),
                     "ERR ARM_CLOSE_REQUIRES_OPEN_FULL STATE=%u OPEN_FULL=0\r\n",
                     (unsigned int)Mechanism_GetState());
            Comm_QueueTx(msg);
            return;
        }

        s_close_armed = 1U;
        Comm_QueueTx("ACK ARM_CLOSE CLOSE_ARMED=1 OPEN_FULL=1\r\n");
        return;
    }


    /* -----------------------------------------------------
     * Normal motion commands remain blocked while any fault
     * is latched.  Recovery commands above remain available.
     * ----------------------------------------------------- */

    if (fault != 0U)
    {
        snprintf(msg,
                 sizeof(msg),
                 "ERR FAULT %u\r\n",
                 fault);

        Comm_QueueTx(msg);
        return;
    }


    if (Mechanism_IsBusy())
    {
        Comm_QueueTx("ERR BUSY\r\n");
        return;
    }


    if (g_mech_cmd != 0U)
    {
        Comm_QueueTx("ERR CMD_PENDING\r\n");
        return;
    }


    if (strcmp(cmd, "OPEN") == 0)
    {
        /* V2.11 mission path: OPEN is direct. If already known full-open,
         * report success instead of rejecting the mission command. */
        if (s_open_full_confirmed ||
            (Mechanism_GetState() == MECH_OPEN_DONE) ||
            (Mechanism_GetState() == MECH_DROP_DONE))
        {
            s_open_full_confirmed = 1U;
            s_closed_full_confirmed = 0U;
            s_close_armed = 0U;
            s_close_resume_allowed = 0U;
            Mechanism_ClampPositionSetOpenFull();
            Comm_QueueTx("OPEN_DONE\r\n");
            return;
        }

        s_close_armed = 0U;
        s_open_full_confirmed = 0U;
        s_closed_full_confirmed = 0U;
        s_close_resume_allowed = 0U;
        g_mech_cmd = 1U;
        Comm_QueueTx("ACK OPEN\r\n");
        return;
    }

    if (strcmp(cmd, "PICK") == 0)
    {
        /* V2.11 SIMPLE PICK: no ARM_CLOSE / OPEN_FULL / CLOSED_FULL /
         * CLOSE_RESUME / POS_PERMILLE permission gate. */
        s_close_armed = 0U;
        s_open_full_confirmed = 0U;
        s_closed_full_confirmed = 0U;
        s_close_resume_allowed = 0U;
        Mechanism_ClampPositionInvalidate();
        g_mech_cmd = 2U;
        Comm_QueueTx("ACK PICK FSR_TARGET=1750 LIFT_TIME_MS=10000\r\n");
        return;
    }

    if (strcmp(cmd, "DROP") == 0)
    {
        s_close_armed = 0U;
        s_open_full_confirmed = 0U;
        s_closed_full_confirmed = 0U;
        s_close_resume_allowed = 0U;
        g_mech_cmd = 3U;
        Comm_QueueTx("ACK DROP\r\n");
        return;
    }

    if (strcmp(cmd, "CLOSE") == 0)
    {
        if (s_closed_full_confirmed)
        {
            Comm_QueueTx("ERR CLOSE_GUARD_ALREADY_CLOSED_FULL CMD=CLOSE CLOSED_FULL=1\r\n");
            return;
        }

        if (!s_close_armed)
        {
            Comm_QueueTx("ERR CLOSE_GUARD_LOCKED CMD=CLOSE\r\n");
            return;
        }

        /* Single-use arm + position confidence are consumed before closing. */
        s_close_armed = 0U;
        s_open_full_confirmed = 0U;
        s_closed_full_confirmed = 0U;
        s_close_resume_allowed = 1U;
        g_mech_cmd = 6U;
        Comm_QueueTx("ACK CLOSE CLOSE_ARMED=0 CLOSE_RESUME=1\r\n");
        return;
    }


    snprintf(msg,
             sizeof(msg),
             "ERR UNKNOWN_CMD CMD=%s\r\n",
             cmd);
    Comm_QueueTx(msg);
}


/* =========================================================
 * USB SESSION RESET - V2.2
 *
 * A host may close/re-open CDC while a partial command line is
 * still buffered.  V2 preserved that fragment, so the first
 * command after reconnect could become e.g. "<stale>PING" and
 * be rejected as UNKNOWN_CMD.
 *
 * This routine clears only parser/session state.  It DOES NOT
 * touch mechanism state, fault latch, or motor outputs.
 * ========================================================= */

static void Comm_ClearRxParserState(void)
{
    s_rx_build_len = 0U;
    s_rx_discard_line = 0U;

    /* Drop any complete commands left from the previous host session. */
    s_rx_head = 0U;
    s_rx_tail = 0U;

    s_rx_overflow_event = 0U;
    s_rx_queue_full_event = 0U;
}


void Communication_USBSessionReset(void)
{
    Comm_ClearRxParserState();

    /* Reconnect must NEVER preserve permission to close.
     * However, reconnect alone does not move the mechanism, so retain
     * the OPEN_FULL physical-position confirmation across host serial
     * sessions. Any real motion, RESET/RESET_FAULT, recovery, or explicit
     * CLEAR_OPEN_FULL still clears it. A MCU reset/reflash also clears it
     * in Communication_Init(). */
    s_close_armed = 0U;

    /* The next real CDC OUT packet gets a second clean boundary.
     * This makes reconnect behavior deterministic even when the
     * host's DTR/control request ordering differs. */
    s_rx_first_packet_after_session = 1U;
}


/* =========================================================
 * RX CALLBACK
 *
 * Called from USB CDC receive callback. Commands end in \n.
 * Multiple lines in a single USB packet are preserved.
 * ========================================================= */

void Communication_USBReceive(uint8_t *data, uint32_t len)
{
    uint32_t i;
    char c;

    if (data == NULL)
        return;

    if (s_rx_first_packet_after_session)
    {
        Comm_ClearRxParserState();
        s_rx_first_packet_after_session = 0U;
    }

    for (i = 0U; i < len; i++)
    {
        c = (char)data[i];

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            if (s_rx_discard_line)
            {
                s_rx_discard_line = 0U;
                s_rx_build_len = 0U;
                continue;
            }

            if (s_rx_build_len > 0U)
            {
                uint8_t head = s_rx_head;
                uint8_t next = (uint8_t)(head + 1U);

                if (next >= COMM_RX_QUEUE_COUNT)
                    next = 0U;

                if (next == s_rx_tail)
                {
                    s_rx_queue_full_event = 1U;
                }
                else
                {
                    uint8_t j;

                    while ((s_rx_build_len > 0U) &&
                           (s_rx_build[s_rx_build_len - 1U] == ' '))
                    {
                        s_rx_build_len--;
                    }

                    if (s_rx_build_len == 0U)
                    {
                        s_rx_build_len = 0U;
                        continue;
                    }

                    s_rx_build[s_rx_build_len] = '\0';

                    for (j = 0U; j < COMM_RX_SIZE; j++)
                    {
                        s_rx_queue[head][j] = s_rx_build[j];

                        if (s_rx_build[j] == '\0')
                            break;
                    }

                    if (j >= COMM_RX_SIZE)
                        s_rx_queue[head][COMM_RX_SIZE - 1U] = '\0';

                    /* Publish the completed slot only after it is written. */
                    s_rx_head = next;
                }
            }

            s_rx_build_len = 0U;
            continue;
        }

        /* Ignore NUL / non-printable control garbage that may appear
         * around CDC re-enumeration.  TAB is normalized to space. */
        if (c == '\t')
            c = ' ';
        else if (((unsigned char)c < 0x20U) || ((unsigned char)c > 0x7EU))
            continue;

        if (s_rx_discard_line)
            continue;

        /* Ignore leading spaces so a reconnect artifact cannot prefix
         * a valid command with whitespace. */
        if ((s_rx_build_len == 0U) && (c == ' '))
            continue;

        if ((c >= 'a') && (c <= 'z'))
        {
            c = (char)(c - ('a' - 'A'));
        }

        if (s_rx_build_len < (COMM_RX_SIZE - 1U))
        {
            s_rx_build[s_rx_build_len] = c;
            s_rx_build_len++;
        }
        else
        {
            /* Discard this entire overlong line until its newline. */
            s_rx_build_len = 0U;
            s_rx_discard_line = 1U;
            s_rx_overflow_event = 1U;
        }
    }
}


/* =========================================================
 * REPORT STATE
 * ========================================================= */

static void Comm_ReportState(void)
{
    uint8_t state;
    uint8_t fault;

    char msg[COMM_TX_SIZE];

    state = (uint8_t)Mechanism_GetState();
    fault = (uint8_t)Mechanism_GetFault();

    /* Synchronize physical-position confidence from states that are only
     * reached after a complete timed motion. */
    if ((state == (uint8_t)MECH_OPEN_DONE) ||
        (state == (uint8_t)MECH_DROP_DONE))
    {
        s_open_full_confirmed = 1U;
        s_closed_full_confirmed = 0U;
        s_close_resume_allowed = 0U;
        Mechanism_ClampPositionSetOpenFull();
    }
    else if ((state == (uint8_t)MECH_CLOSING) ||
             (state == (uint8_t)MECH_CLOSE_DONE) ||
             (state == (uint8_t)MECH_CLAMPING) ||
             (state == (uint8_t)MECH_LIFTING) ||
             (state == (uint8_t)MECH_PICK_DONE) ||
             (state == (uint8_t)MECH_LOWERING) ||
             (state == (uint8_t)MECH_RELEASING) ||
             (state == (uint8_t)MECH_RECOVER_OPENING) ||
             (state == (uint8_t)MECH_JOG_CLOSING) ||
             (state == (uint8_t)MECH_JOG_OPENING) ||
             (state == (uint8_t)MECH_JOG_LIFT_UP) ||
             (state == (uint8_t)MECH_JOG_LIFT_DOWN) ||
             (state == (uint8_t)MECH_OPENING))
    {
        s_open_full_confirmed = 0U;
        s_closed_full_confirmed = 0U;
    }

    if (state == (uint8_t)MECH_CLOSING)
    {
        s_close_resume_allowed = 1U;
    }
    else if ((state == (uint8_t)MECH_CLOSE_DONE) ||
             (state == (uint8_t)MECH_OPENING) ||
             (state == (uint8_t)MECH_OPEN_DONE) ||
             (state == (uint8_t)MECH_RECOVER_OPENING) ||
             (state == (uint8_t)MECH_FAULT) ||
             (state == (uint8_t)MECH_CLAMPING) ||
             (state == (uint8_t)MECH_LIFTING) ||
             (state == (uint8_t)MECH_PICK_DONE) ||
             (state == (uint8_t)MECH_LOWERING) ||
             (state == (uint8_t)MECH_RELEASING) ||
             (state == (uint8_t)MECH_DROP_DONE))
    {
        s_close_resume_allowed = 0U;
    }

    if ((state == s_prev_state) &&
        (fault == s_prev_fault))
    {
        return;
    }

    s_prev_state = state;
    s_prev_fault = fault;

    switch ((MechanismState_t)state)
    {
        case MECH_IDLE:
            Comm_QueueTx("STATE IDLE\r\n");
            break;

        case MECH_OPENING:
            Comm_QueueTx("STATE OPENING\r\n");
            break;

        case MECH_OPEN_DONE:
            Comm_QueueTx("OPEN_DONE\r\n");
            break;

        case MECH_CLOSING:
            Comm_QueueTx("STATE CLOSING\r\n");
            break;

        case MECH_CLOSE_DONE:
            Comm_QueueTx("CLOSE_DONE\r\n");
            break;

        case MECH_JOG_CLOSING:
            Comm_QueueTx("STATE JOG_CLOSING\r\n");
            break;

        case MECH_JOG_OPENING:
            Comm_QueueTx("STATE JOG_OPENING\r\n");
            break;

        case MECH_JOG_LIFT_UP:
            Comm_QueueTx("STATE JOG_LIFT_UP\r\n");
            break;
        case MECH_JOG_LIFT_DOWN:
            Comm_QueueTx("STATE JOG_LIFT_DOWN\r\n");
            break;

        case MECH_CLAMPING:
            Comm_QueueTx("STATE CLAMPING\r\n");
            break;

        case MECH_LIFTING:
            Comm_QueueTx("STATE LIFTING\r\n");
            break;

        case MECH_PICK_DONE:
            Comm_QueueTx("PICK_DONE\r\n");
            break;

        case MECH_LOWERING:
            Comm_QueueTx("STATE LOWERING\r\n");
            break;

        case MECH_RELEASING:
            Comm_QueueTx("STATE RELEASING\r\n");
            break;

        case MECH_DROP_DONE:
            Comm_QueueTx("DROP_DONE\r\n");
            break;

        case MECH_RECOVER_OPENING:
            snprintf(msg,
                     sizeof(msg),
                     "STATE RECOVER_OPENING FAULT=%u\r\n",
                     fault);
            Comm_QueueTx(msg);
            break;

        case MECH_FAULT:
            snprintf(msg,
                     sizeof(msg),
                     "FAULT %u RECOVER_OPEN=%u\r\n",
                     fault,
                     Mechanism_CanRecoverOpen());
            Comm_QueueTx(msg);
            break;

        default:
            snprintf(msg,
                     sizeof(msg),
                     "STATE %u\r\n",
                     state);
            Comm_QueueTx(msg);
            break;
    }
}


/* =========================================================
 * INIT
 * ========================================================= */

void Communication_Init(void)
{
    Communication_USBSessionReset();

    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_tx_count = 0U;

    s_prev_state = 255U;
    s_prev_fault = 255U;
    s_close_armed = 0U;
    s_open_full_confirmed = 0U;
    s_closed_full_confirmed = 0U;
    s_close_resume_allowed = 0U;

    /* V2.2 protocol is strictly request/response.
     * Do not emit an unsolicited startup banner: some host terminal
     * stacks may locally echo startup text and contaminate the first
     * command line. VERSION/STATUS provide explicit identification. */
}


/* =========================================================
 * MAIN TASK
 * ========================================================= */

void Communication_Task(void)
{
    char cmd[COMM_RX_SIZE];

    /* Report RX failures from the main context, never USB ISR. */
    if (s_rx_overflow_event)
    {
        s_rx_overflow_event = 0U;
        Comm_QueueTx("ERR RX_OVERFLOW\r\n");
    }

    if (s_rx_queue_full_event)
    {
        s_rx_queue_full_event = 0U;
        Comm_QueueTx("ERR RX_QUEUE_FULL\r\n");
    }

    /* Process at most one command per main-loop pass. */
    if (Comm_PopRxLine(cmd))
    {
        Comm_ProcessCommand(cmd);
    }

    Comm_ReportState();
    Comm_TxTask();
}
