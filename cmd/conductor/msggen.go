package main

import (
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"strings"

	"conductor.dev/conductor/internal/msggen"
)

func runMsggen(args []string) error {
	fs := flag.NewFlagSet("msggen", flag.ExitOnError)
	out := fs.String("out", "", "output directory (required)")
	goPkg := fs.String("pkg", "", "Go package name (default: basename of -out)")
	rosPkg := fs.String("ros-pkg", "", "ROS package name for local .msg file/dir targets")
	fs.Parse(args)
	if *out == "" || fs.NArg() == 0 {
		return fmt.Errorf("msggen: -out and at least one target are required (see conductor -h)")
	}
	if *goPkg == "" {
		*goPkg = filepath.Base(*out)
	}

	r := msggen.NewResolver(msggen.SharePrefixesFromEnv(os.Getenv("AMENT_PREFIX_PATH")))
	var targets []string
	for _, arg := range fs.Args() {
		switch {
		case strings.HasSuffix(arg, ".msg") || strings.HasSuffix(arg, ".srv") || strings.HasSuffix(arg, ".action"):
			full, err := addLocal(r, *rosPkg, arg)
			if err != nil {
				return err
			}
			targets = append(targets, full)
		case isDir(arg):
			var files []string
			for _, pattern := range []string{"*.msg", "*.srv", "*.action"} {
				matched, err := filepath.Glob(filepath.Join(arg, pattern))
				if err != nil {
					return err
				}
				files = append(files, matched...)
			}
			if len(files) == 0 {
				return fmt.Errorf("msggen: no .msg/.srv/.action files in %s", arg)
			}
			for _, f := range files {
				full, err := addLocal(r, *rosPkg, f)
				if err != nil {
					return err
				}
				targets = append(targets, full)
			}
		default:
			parts := strings.Split(arg, "/")
			switch len(parts) {
			case 2:
				targets = append(targets, parts[0]+"/msg/"+parts[1])
			case 3:
				targets = append(targets, arg)
			default:
				return fmt.Errorf("msggen: target %q is neither a .msg path nor a pkg/msg/Name interface name", arg)
			}
		}
	}

	src, err := msggen.Generate(r, targets, *goPkg)
	if err != nil {
		return err
	}
	if err := os.MkdirAll(*out, 0o755); err != nil {
		return err
	}
	dest := filepath.Join(*out, *goPkg+"_msggen.go")
	if err := os.WriteFile(dest, src, 0o644); err != nil {
		return err
	}
	fmt.Printf("wrote %s (%d target(s))\n", dest, len(targets))
	return nil
}

func addLocal(r *msggen.Resolver, rosPkg, path string) (string, error) {
	if rosPkg == "" {
		return "", fmt.Errorf("msggen: local target %s requires -ros-pkg", path)
	}
	return r.AddLocal(rosPkg, path)
}

func isDir(p string) bool {
	fi, err := os.Stat(p)
	return err == nil && fi.IsDir()
}
