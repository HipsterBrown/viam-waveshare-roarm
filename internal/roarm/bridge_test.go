package roarm

import "testing"

func TestWaitArg(t *testing.T) {
	for _, tc := range []struct {
		name string
		in   map[string]interface{}
		want bool
	}{
		{"nil defaults to waiting", nil, true},
		{"empty defaults to waiting", map[string]interface{}{}, true},
		{"waitAtEnd false", map[string]interface{}{"waitAtEnd": false}, false},
		{"wait false alias", map[string]interface{}{"wait": false}, false},
		{"waitAtEnd true", map[string]interface{}{"waitAtEnd": true}, true},
		// The RDK-originated spelling wins, so a caller sending both cannot be
		// silently given the opposite of what the motion service asked for.
		{"waitAtEnd wins over wait", map[string]interface{}{"waitAtEnd": false, "wait": true}, false},
		{"wrong type is ignored", map[string]interface{}{"waitAtEnd": "false"}, true},
	} {
		if got := WaitArg(tc.in); got != tc.want {
			t.Errorf("%s: WaitArg(%v) = %v, want %v", tc.name, tc.in, got, tc.want)
		}
	}
}

func TestInterpolateArg(t *testing.T) {
	for _, tc := range []struct {
		name string
		in   map[string]interface{}
		want bool
	}{
		{"nil defaults to interpolating", nil, true},
		{"explicit false", map[string]interface{}{"interpolate": false}, false},
		{"explicit true", map[string]interface{}{"interpolate": true}, true},
		{"wrong type is ignored", map[string]interface{}{"interpolate": 0}, true},
	} {
		if got := InterpolateArg(tc.in); got != tc.want {
			t.Errorf("%s: InterpolateArg(%v) = %v, want %v", tc.name, tc.in, got, tc.want)
		}
	}
}
