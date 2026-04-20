package waveshareroarm

import (
	"encoding/json"
	"fmt"
	"time"
)

// Duration wraps time.Duration so it can be configured as either a
// human-readable string (e.g. "5s") or a raw nanosecond integer.
type Duration time.Duration

func (d *Duration) UnmarshalJSON(data []byte) error {
	if len(data) == 0 || string(data) == "null" {
		return nil
	}
	// Try integer nanoseconds first.
	var n int64
	if err := json.Unmarshal(data, &n); err == nil {
		*d = Duration(n)
		return nil
	}
	var s string
	if err := json.Unmarshal(data, &s); err != nil {
		return fmt.Errorf("duration must be string or number: %w", err)
	}
	if s == "" {
		*d = 0
		return nil
	}
	parsed, err := time.ParseDuration(s)
	if err != nil {
		return fmt.Errorf("invalid duration %q: %w", s, err)
	}
	*d = Duration(parsed)
	return nil
}

func (d Duration) MarshalJSON() ([]byte, error) {
	return json.Marshal(time.Duration(d).String())
}

func (d Duration) ToStdDuration() time.Duration { return time.Duration(d) }
