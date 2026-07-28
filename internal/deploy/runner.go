package deploy

import (
	"fmt"
	"io"
	"os"
	"os/exec"
	"strings"
)

// Runner executes the deployment's few commands either on this machine or on
// a robot over ssh. Command construction is separate from execution so the
// remote path can be tested without a remote.
type Runner struct {
	Host   string // empty: this machine
	Sudo   string // privilege prefix, e.g. "sudo -n"; empty for none
	DryRun bool
	Out    io.Writer
}

// CopyCmd returns the command that puts a local file on the target.
func (r Runner) CopyCmd(local, remote string) *exec.Cmd {
	if r.Host == "" {
		return exec.Command("cp", local, remote)
	}
	return exec.Command("scp", "-q", local, r.Host+":"+remote)
}

// ScriptCmd returns the command that runs a shell script on the target. The
// script is fed on stdin rather than passed as an argument: it keeps quoting
// to one level, and it is what makes the local and remote forms identical.
func (r Runner) ScriptCmd(script string) *exec.Cmd {
	if r.Sudo != "" {
		script = wrapSudo(script, r.Sudo)
	}
	var cmd *exec.Cmd
	if r.Host == "" {
		cmd = exec.Command("bash", "-s")
	} else {
		cmd = exec.Command("ssh", "-o", "BatchMode=yes", r.Host, "bash -s")
	}
	cmd.Stdin = strings.NewReader(script)
	return cmd
}

// wrapSudo re-runs the whole script under the privilege prefix, so a script
// with several privileged steps needs one authorization, not one per line.
func wrapSudo(script, sudo string) string {
	return sudo + " bash -s <<'CONDUCTOR_EOF'\n" + script + "\nCONDUCTOR_EOF\n"
}

func (r Runner) Copy(local, remote string) error {
	return r.run(r.CopyCmd(local, remote), fmt.Sprintf("copy %s -> %s", local, r.at(remote)))
}

func (r Runner) Script(script string) error {
	// A dry run must show what actually runs, privilege wrapper included.
	shown := script
	if r.Sudo != "" {
		shown = wrapSudo(script, r.Sudo)
	}
	return r.run(r.ScriptCmd(script), "run"+r.where()+":\n"+indent(shown))
}

func (r Runner) where() string {
	if r.Host == "" {
		return " here"
	}
	return " on " + r.Host
}

func (r Runner) run(cmd *exec.Cmd, what string) error {
	if r.DryRun {
		fmt.Fprintf(r.Out, "would %s\n", what)
		return nil
	}
	cmd.Stdout, cmd.Stderr = r.Out, os.Stderr
	if err := cmd.Run(); err != nil {
		return fmt.Errorf("%s: %w", cmd.Args[0], err)
	}
	return nil
}

func (r Runner) at(p string) string {
	if r.Host == "" {
		return p
	}
	return r.Host + ":" + p
}

func indent(s string) string {
	lines := strings.Split(strings.TrimRight(s, "\n"), "\n")
	for i, l := range lines {
		lines[i] = "    " + l
	}
	return strings.Join(lines, "\n")
}

// shJoin quotes arguments for a shell command line.
func shJoin(args []string) string {
	out := make([]string, len(args))
	for i, a := range args {
		out[i] = shQuote(a)
	}
	return strings.Join(out, " ")
}

func shQuote(s string) string {
	if s == "" {
		return "''"
	}
	for _, r := range s {
		safe := r == '-' || r == '_' || r == '.' || r == '/' || r == ':' || r == '=' ||
			(r >= 'a' && r <= 'z') || (r >= 'A' && r <= 'Z') || (r >= '0' && r <= '9')
		if !safe {
			return "'" + strings.ReplaceAll(s, "'", `'\''`) + "'"
		}
	}
	return s
}
