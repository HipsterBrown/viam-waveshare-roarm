package roarm

import (
	"testing"
	"time"

	"go.viam.com/rdk/logging"
)

// A link that retries more than 5% of its reads, over enough reads for the
// ratio to mean anything, warns once and then stays quiet.
func TestRetryWatchdogWarnsOnceThenRateLimits(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	c := &Controller{logger: logger}
	c.health.Frames = healthMinSamples
	c.health.Retries = healthMinSamples // 100%

	c.checkLinkHealth()
	c.checkLinkHealth()
	if n := logs.FilterMessageSnippet("retried").Len(); n != 1 {
		t.Fatalf("want exactly one warning, got %d", n)
	}
	if logs.FilterMessageSnippet("comms_health").Len() != 1 {
		t.Fatal("the warning should point at the comms_health command")
	}

	// Past the interval it may warn again, because the problem persists and the
	// counters are cumulative.
	c.health.lastWarn = time.Now().Add(-2 * healthWarnInterval)
	c.checkLinkHealth()
	if n := logs.FilterMessageSnippet("retried").Len(); n != 2 {
		t.Fatalf("the watchdog should warn again after its interval, got %d warnings", n)
	}
}

// Below healthMinSamples a single retry is not evidence of anything.
func TestRetryWatchdogIsQuietOnASmallSample(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	c := &Controller{logger: logger}
	c.health.Frames = healthMinSamples - 1
	c.health.Retries = healthMinSamples - 1
	c.checkLinkHealth()
	if logs.FilterMessageSnippet("retried").Len() != 0 {
		t.Fatalf("warned on a %d-read sample", c.health.Frames)
	}
}

// A healthy link never warns, whatever its absolute retry count.
func TestRetryWatchdogIsQuietOnAHealthyLink(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	c := &Controller{logger: logger}
	c.health.Frames = 10000
	c.health.Retries = 400 // 4%, under the threshold
	c.checkLinkHealth()
	if logs.FilterMessageSnippet("retried").Len() != 0 {
		t.Fatal("warned on a 4% retry rate")
	}
}

func TestHealthMapAndReset(t *testing.T) {
	c := &Controller{}
	c.health.Frames = 200
	c.health.Retries = 20
	m := c.Health().Map()
	if m["frames"] != 200 || m["retry_pct"] != 10.0 {
		t.Fatalf("health map: %v", m)
	}
	if _, ok := m["last_error"]; ok {
		t.Fatal("last_error should be absent until something fails")
	}
	c.ResetHealth()
	if c.Health().Frames != 0 {
		t.Fatal("reset left counters behind")
	}
}
