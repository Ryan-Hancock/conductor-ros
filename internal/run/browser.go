package run

import (
	"fmt"
	"os"
	"os/exec"
	"runtime"
)

// Opening a browser is the last small courtesy of a development loop, and the
// place to be careful: the same command runs over ssh to a robot, in CI, and
// on a machine with no desktop at all. So the rule is to open only where
// there is plainly something to open, and to print the URL either way — the
// URL is the part that matters, the browser is the convenience.

// openBrowser opens url when this looks like a desktop session, unless the
// caller has said otherwise.
func (s *Session) openBrowser(url string) {
	want := desktop()
	if s.opts.Open != nil {
		want = *s.opts.Open
	}
	if !want {
		return
	}
	cmd, err := browserCommand(url)
	if err != nil {
		fmt.Fprintf(s.out, "conductor: open it yourself: %v\n", err)
		return
	}
	cmd.Stdout, cmd.Stderr = nil, nil
	if err := cmd.Start(); err != nil {
		fmt.Fprintf(s.out, "conductor: could not open a browser (%v)\n", err)
		return
	}
	go cmd.Wait() // reap it; a browser outlives this run
}

// desktop reports whether opening a browser is likely to do something. An ssh
// session or a CI run is exactly where it would not.
func desktop() bool {
	if os.Getenv("CI") != "" || os.Getenv("SSH_CONNECTION") != "" || os.Getenv("SSH_TTY") != "" {
		return false
	}
	switch runtime.GOOS {
	case "darwin", "windows":
		return true
	}
	// Linux, including WSL: a display, or a Windows host to hand it to.
	if os.Getenv("DISPLAY") != "" || os.Getenv("WAYLAND_DISPLAY") != "" {
		return true
	}
	if _, err := exec.LookPath("wslview"); err == nil {
		return true
	}
	return false
}

// browserCommand is how this platform opens a URL.
func browserCommand(url string) (*exec.Cmd, error) {
	var candidates [][]string
	switch runtime.GOOS {
	case "darwin":
		candidates = [][]string{{"open", url}}
	case "windows":
		candidates = [][]string{{"rundll32", "url.dll,FileProtocolHandler", url}}
	default:
		candidates = [][]string{
			{"xdg-open", url},
			{"wslview", url}, // WSL: hands the URL to the Windows host
			{"gio", "open", url},
		}
	}
	for _, c := range candidates {
		if path, err := exec.LookPath(c[0]); err == nil {
			return exec.Command(path, c[1:]...), nil
		}
	}
	return nil, fmt.Errorf("no way to open a browser here")
}
