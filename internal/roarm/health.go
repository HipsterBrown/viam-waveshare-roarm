package roarm

import (
	"errors"
	"time"
)

// Link health counters. Deliberately small: cumulative totals, no sliding
// window, and nothing in the control path branches on them. They answer one
// question from the bench or from a running machine: is this link losing
// frames, and has it been losing them all along or only since something
// changed? Per-minute rates are omitted until the cumulative ratio proves too
// blunt to spot a fraying cable.
//
// Every counter is mutated under Controller.mu. The transport already holds it
// for the whole of write and query; the settle counters take it in noteSettle.
const (
	// healthMinSamples is how many reads must have happened before the retry
	// ratio means anything: below it one retry looks like a catastrophe.
	healthMinSamples = 100
	// retryRateWarnPct is the retry-to-read percentage above which the link is
	// worth warning about.
	retryRateWarnPct = 5
	// healthWarnInterval rate-limits that warning.
	healthWarnInterval = time.Minute
)

// HealthSnapshot is a copy of the counters, safe to hand to a caller.
type HealthSnapshot struct {
	Frames           int // frames read and validated
	ReadTimeouts     int // reads that produced no usable frame in time
	InvalidFrames    int // candidate frames rejected as incomplete or corrupt
	TransportErrors  int // errors retrying cannot fix: a closed port, a short write
	Retries          int // read attempts beyond the first
	RetriesExhausted int // reads that failed every attempt
	// StaleFrames counts well-formed frames carrying a T the module did not
	// ask for: traffic it is reading but never requested (audit 2.8).
	StaleFrames    int
	ResetFailures  int // ResetInputBuffer failures
	ShortWrites    int
	SettlesArrived int
	SettlesStopped int
	SettleTimeouts int
	NeverMoved     int
	LastError      string
	LastErrorAt    time.Time
	// lastWarn is when checkLinkHealth last warned. Unexported, so it is
	// copied along with the rest of HealthSnapshot but never escapes: Map
	// ignores it.
	lastWarn time.Time
}

// RetryPct is retries as a percentage of frames read: the single number worth
// watching. 0 when nothing has been read yet.
func (h HealthSnapshot) RetryPct() float64 {
	if h.Frames == 0 {
		return 0
	}
	return 100 * float64(h.Retries) / float64(h.Frames)
}

// Health returns a copy of the counters.
func (c *Controller) Health() HealthSnapshot {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.health
}

// retryCount reads just the retry total, which WaitUntilSettled samples either
// side of a settle to report how many of its polls needed one.
func (c *Controller) retryCount() int {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.health.Retries
}

// noteError records the most recent failure, so comms_health can say what went
// wrong without the caller digging through logs.
func (c *Controller) noteError(err error) {
	c.health.LastError = err.Error()
	c.health.LastErrorAt = time.Now()
}

// noteSettle records how a settle ended. The outcome counters are what make a
// bench run comparable across sessions: a link that used to arrive every time
// and now stops short has changed, even when every individual move looked
// acceptable.
func (c *Controller) noteSettle(res SettleResult, err error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	switch {
	case err == nil && res.Outcome == SettleArrived:
		c.health.SettlesArrived++
	case err == nil:
		c.health.SettlesStopped++
	case errors.Is(err, ErrArmDidNotMove):
		c.health.NeverMoved++
		c.noteError(err)
	default:
		c.health.SettleTimeouts++
		c.noteError(err)
	}
}

// Map renders the counters for the comms_health DoCommand. Keys are snake_case
// to match the rest of the module's DoCommand vocabulary.
func (h HealthSnapshot) Map() map[string]interface{} {
	m := map[string]interface{}{
		"frames":            h.Frames,
		"read_timeouts":     h.ReadTimeouts,
		"invalid_frames":    h.InvalidFrames,
		"transport_errors":  h.TransportErrors,
		"retries":           h.Retries,
		"retries_exhausted": h.RetriesExhausted,
		"retry_pct":         h.RetryPct(),
		"stale_frames":      h.StaleFrames,
		"reset_failures":    h.ResetFailures,
		"short_writes":      h.ShortWrites,
		"settles_arrived":   h.SettlesArrived,
		"settles_stopped":   h.SettlesStopped,
		"settle_timeouts":   h.SettleTimeouts,
		"never_moved":       h.NeverMoved,
	}
	if h.LastError != "" {
		m["last_error"] = h.LastError
		m["last_error_at"] = h.LastErrorAt.Format(time.RFC3339)
	}
	return m
}

// ResetHealth zeroes the counters, so a bench run can measure one experiment
// rather than the whole session.
func (c *Controller) ResetHealth() {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.health = HealthSnapshot{}
}

// checkLinkHealth warns when the cumulative retry ratio says the link is
// lossy. One warning per healthWarnInterval: a fraying cable would otherwise
// fill the log at the poll rate, and the counters are cumulative, so the
// condition stays true once it is true.
//
// Callers must not hold c.mu: this takes it itself. query holds c.mu for its
// whole body under a defer, so this is called from GetFeedback and
// GetJointRadians after query returns, never from inside it — a Go mutex is
// not reentrant, and calling this from inside query would deadlock.
func (c *Controller) checkLinkHealth() {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.health.Frames < healthMinSamples || c.health.RetryPct() <= retryRateWarnPct {
		return
	}
	if time.Since(c.health.lastWarn) < healthWarnInterval {
		return
	}
	c.health.lastWarn = time.Now()
	c.logger.Warnf("this link retried %.1f%% of its %d feedback reads (%d retries, %d reads failed outright); "+
		"check the cable and connector, and run the comms_health command for the full counters",
		c.health.RetryPct(), c.health.Frames, c.health.Retries, c.health.RetriesExhausted)
}
