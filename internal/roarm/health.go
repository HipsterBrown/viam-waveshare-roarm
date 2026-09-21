package roarm

import "time"

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
	InvalidFrames    int // frames rejected as incomplete, corrupt, or the wrong T
	TransportErrors  int // errors retrying cannot fix: a closed port, a short write
	Retries          int // read attempts beyond the first
	RetriesExhausted int // reads that failed every attempt
	StaleFrames      int // frames discarded because a newer one followed them
	ResetFailures    int // ResetInputBuffer failures
	ShortWrites      int
	SettlesArrived   int
	SettlesStopped   int
	SettleTimeouts   int
	NeverMoved       int
	LastError        string
	LastErrorAt      time.Time
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
