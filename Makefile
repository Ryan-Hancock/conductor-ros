# Conductor — development tasks.
#
#   make            what you can run
#   make test       unit tests (no ROS install needed)
#   make verify     fmt + vet + test + graph validation of every example
#   make interop    the full matrix against real ROS 2 over rmw_zenoh
#
# Targets tagged "needs ROS" source .tools/env.sh for the user-space
# rmw_zenoh overlay and the cgo flags for zenoh-c; everything else builds
# with a plain Go toolchain.

SHELL := /bin/bash
.SHELLFLAGS := -o pipefail -c

ENV      := .tools/env.sh
EXAMPLES := chatter patrol fibonacci mission turtlesim
CLI      := go run ./cmd/conductor
# ROS setup.bash reads unset variables, so env.sh must not run under `set -u`.
WITHROS   = source $(ENV) &&

.DEFAULT_GOAL := help

.PHONY: help
help: ## show this help
	@echo "Conductor targets:"
	@grep -hE '^[a-z][a-zA-Z0-9_-]*:.*?## ' $(MAKEFILE_LIST) \
	  | awk -F':.*?## ' '{printf "  \033[36m%-16s\033[0m %s\n", $$1, $$2}'
	@echo "  interop-<group>  (ROS) one group: lifecycle params services actions frames turtlesim"
	@echo
	@echo "Targets marked (ROS) need $(ENV) — see README."

# ---------------------------------------------------------------- build/test

.PHONY: build
build: ## compile everything with the default in-process transport
	go build ./...

.PHONY: test
test: ## unit tests (runtime, harness, examples, CDR, graph, RIHS01 corpus)
	go test ./...

.PHONY: test-race
test-race: ## unit tests under the race detector
	go test -race ./...

.PHONY: fmt
fmt: ## gofmt the tree
	gofmt -l -w .

.PHONY: vet
vet: ## go vet, including the cgo transport
	go vet ./...
	@if [[ -f $(ENV) ]]; then $(WITHROS) go vet -tags zenoh ./...; \
	 else echo "skipping -tags zenoh vet (no $(ENV))"; fi

.PHONY: check
check: ## validate every example's graph (conductor check)
	@for ex in $(EXAMPLES); do \
	  echo "== examples/$$ex"; \
	  $(CLI) check examples/$$ex || exit 1; \
	done

.PHONY: check-envs
check-envs: ## validate examples/patrol in each declared environment
	@for env in sim bench robot; do \
	  echo "== env $$env"; \
	  $(CLI) check examples/patrol -env $$env | tail -3 || exit 1; \
	done

.PHONY: bundle
bundle: ## build a release bundle for examples/patrol (no target touched)
	$(CLI) deploy examples/patrol -env bench -bundle

.PHONY: deploy-dry
deploy-dry: ## (ROS) show what deploying examples/patrol to its robot would run
	@$(WITHROS) $(CLI) deploy examples/patrol -env robot -dry-run -goarch $$(go env GOARCH) -cc gcc

.PHONY: verify
verify: fmt vet test-race check ## everything that runs without a live ROS graph

.PHONY: dashboard
dashboard: ## run examples/patrol with the live dashboard on :4000
	cd examples/patrol && go run . -dashboard 127.0.0.1:4000 -dashboard-traces 500

.PHONY: fleet
fleet: ## (ROS) run examples/patrol as four zenoh processes behind the fleet view
	@$(WITHROS) set -m; \
	  pkill -9 -x patrol 2>/dev/null; pkill -9 -x rmw_zenohd 2>/dev/null; sleep 1; \
	  go build -tags zenoh -o bin/patrol ./examples/patrol || exit 1; \
	  $$CONDUCTOR_OVERLAY/lib/rmw_zenoh_cpp/rmw_zenohd >/dev/null 2>&1 & router=$$!; \
	  sleep 2; \
	  i=0; for n in localizer patroller navigator safety_monitor; do \
	    (cd examples/patrol && ../../bin/patrol -transport zenoh -node $$n \
	      -dashboard 127.0.0.1:$$((4000+i)) >/dev/null 2>&1 &); \
	    i=$$((i+1)); \
	  done; \
	  sleep 3; \
	  $(CLI) dashboard examples/patrol -env robot -host 127.0.0.1; \
	  pkill -9 -x patrol 2>/dev/null; kill -9 $$router 2>/dev/null

.PHONY: gen
gen: ## regenerate launch XML, params, graph/mission/frames dot for examples/patrol
	$(CLI) build examples/patrol

.PHONY: mission
mission: ## (ROS) run the declarative mission example against the Go action server
	@$(WITHROS) set -m; \
	  pkill -9 -x rmw_zenohd 2>/dev/null; sleep 1; \
	  $$CONDUCTOR_OVERLAY/lib/rmw_zenoh_cpp/rmw_zenohd >/dev/null 2>&1 & router=$$!; \
	  sleep 2; \
	  go run -tags zenoh ./examples/fibonacci -transport zenoh >/dev/null 2>&1 & server=$$!; \
	  sleep 2; \
	  go run -tags zenoh ./examples/mission -transport zenoh; rc=$$?; \
	  kill -9 $$server $$router 2>/dev/null; \
	  exit $$rc

.PHONY: clean
clean: ## remove build output
	rm -rf examples/*/gen bin
	go clean -cache -testcache

# --------------------------------------------------------------- ROS / zenoh

.PHONY: env
env: ## (ROS) print the overlay environment this Makefile uses
	@$(WITHROS) env | grep -E '^(RMW_IMPLEMENTATION|AMENT_PREFIX_PATH|CONDUCTOR_OVERLAY|ZENOH_C|CGO_)'

.PHONY: build-zenoh
build-zenoh: ## (ROS) compile with the cgo zenoh transport
	$(WITHROS) go build -tags zenoh ./...

.PHONY: test-zenoh
test-zenoh: ## (ROS) unit tests with the zenoh transport compiled in
	$(WITHROS) go test -tags zenoh ./...

.PHONY: router
router: ## (ROS) run the zenoh router in the foreground
	$(WITHROS) exec $$CONDUCTOR_OVERLAY/lib/rmw_zenoh_cpp/rmw_zenohd

.PHONY: interop
interop: ## (ROS) full interop matrix against real ROS 2 (starts its own router)
	./.tools/interop.sh

# make interop-services / -actions / -lifecycle / -params / -turtlesim
.PHONY: interop-%
interop-%: ## (ROS) one interop group
	./.tools/interop.sh $*

.PHONY: turtlesim
turtlesim: ## (ROS) run the turtlesim tutorial: router + turtlesim_node + the example
	@$(WITHROS) set -m; \
	  pkill -9 -x turtlesim_node 2>/dev/null; pkill -9 -x rmw_zenohd 2>/dev/null; sleep 1; \
	  $$CONDUCTOR_OVERLAY/lib/rmw_zenoh_cpp/rmw_zenohd >/dev/null 2>&1 & router=$$!; \
	  sleep 2; \
	  ros2 run turtlesim turtlesim_node >/dev/null 2>&1 & sleep 4; \
	  go run -tags zenoh ./examples/turtlesim -transport zenoh; rc=$$?; \
	  pkill -9 -x turtlesim_node 2>/dev/null; kill -9 $$router 2>/dev/null; \
	  exit $$rc
