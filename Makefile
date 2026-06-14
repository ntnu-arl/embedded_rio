# Convenience wrapper for the offline replay tooling.
#
# Default inputs come from environment variables RIO_LOG / RIO_ULG (e.g.
# put `export RIO_LOG=...` in your shell rc). If those are unset, the
# hard-coded fallbacks below are used. Either can still be overridden on
# the command line, which takes top priority:
#
#     export RIO_LOG=data/field/2105/csv/LOG0007.CSV   # session-wide
#     make run                                         # uses $RIO_LOG
#     make run LOG=some/other.CSV                      # one-off override
#     make compare ARGS=--no-vvo                       # forward flags
#     make all run compare                             # full chain

LOG   ?= $(or $(RIO_LOG),data/field/2105/csv/LOG0012.CSV)
ULG   ?= $(or $(RIO_ULG),data/field/2105/ulog/log_24_2026-5-21-20-47-18.ulg)
MOCAP ?= $(RIO_MOCAP)

# Report where a variable's value came from. Used by `make help`.
# Args: $1 = variable name (e.g. LOG), $2 = env var name (e.g. RIO_LOG).
src = $(if $(filter command line,$(origin $1)),CLI override,$(if $($2),from \$$$2,fallback))
NAME  ?= $(notdir $(basename $(LOG)))
OUT   ?= runs/$(NAME)
PY    ?= .venv/bin/python3
ARGS  ?=

BUILD_DIR := build/replay
BIN       := $(BUILD_DIR)/rio_replay
CACHE     := $(BUILD_DIR)/CMakeCache.txt

# Shortcuts that translate to compare_with_ulog.py flags via $(ARGS).
# Anything else can be passed verbatim, e.g. ARGS="--tol 0.3 --no-vvo".
ifdef NO_VVO
  ARGS += --no-vvo
endif
ifdef TOL
  ARGS += --tol $(TOL)
endif
ifdef SKIP
  ARGS += --skip-seconds $(SKIP)
endif
ifdef ALIGN_EKF2
  ARGS += --align-to-ekf2
endif
ifdef NO_MOUNT
  ARGS += --no-mount-correction
endif
ifneq ($(strip $(MOCAP)),)
  ARGS += --mocap "$(MOCAP)"
endif
ifdef MOCAP_OFFSET
  ARGS += --mocap-offset $(MOCAP_OFFSET)
endif
ifdef MOCAP_SYNC
  ARGS += --mocap-sync-with $(MOCAP_SYNC)
endif
ifdef MOCAP_FRAME
  ARGS += --mocap-body-frame $(MOCAP_FRAME)
endif

.PHONY: all build run compare ablation clean help

all: build

# CMake configure step — only re-runs if the top-level CMakeLists.txt changes.
$(CACHE): tools/replay/CMakeLists.txt
	cmake -S tools/replay -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=Release

# Incremental build via cmake --build (Ninja/Make underneath handles deps).
build: $(CACHE)
	cmake --build $(BUILD_DIR) -j

$(BIN): build

# Aiding flags (baro on/off, differential vs absolute, radar on/off) live
# in src/config.h — shared with firmware. Edit there and rebuild to switch.
RUN_ARGS :=

run: $(BIN)
	mkdir -p "$(OUT)"
	$(BIN) $(RUN_ARGS) "$(LOG)" "$(OUT)"
	@echo "→ $(OUT)/"

compare:
	@test -x $(PY) || { echo "missing $(PY) — create a venv with 'python3 -m venv .venv && .venv/bin/pip install evo pyulog matplotlib pandas scipy'"; exit 1; }
	$(PY) tools/replay/scripts/compare_with_ulog.py "$(OUT)" "$(ULG)" $(ARGS)
	$(PY) tools/replay/scripts/plot_runs.py "$(OUT)" --save

# Baro×underweighting ablation: builds rio_replay 4× (one per flag combo),
# replays LOG through each into runs/$(NAME)/ablation/<tag>/, then overlays +
# summarizes them in runs/$(NAME)/ablation/cmp/. GT = mocap when MOCAP is set,
# else EKF2 (from ULG). SKIP=… is forwarded to compare_ablation.py.
ablation:
	@test -x $(PY) || { echo "missing $(PY) — see 'make compare' note for the venv"; exit 1; }
	LOG="$(LOG)" ULG="$(ULG)" NAME="$(NAME)" SKIP="$(SKIP)" PY="$(PY)" \
	  MOCAP="$(MOCAP)" MOCAP_OFFSET="$(MOCAP_OFFSET)" MOCAP_FRAME="$(MOCAP_FRAME)" \
	  NO_MOUNT="$(NO_MOUNT)" \
	  bash tools/replay/scripts/run_ablation.sh

clean:
	rm -rf $(BUILD_DIR)

help:
	@echo "targets:"
	@echo "  make all       — configure + build rio_replay (default)"
	@echo "  make run       — replay LOG into OUT"
	@echo "  make compare   — run compare_with_ulog.py OUT vs ULG"
	@echo "  make ablation  — build+run 4 baro×underweighting configs into runs/NAME/ablation/ (needs LOG + ULG; MOCAP optional GT)"
	@echo "  make clean     — remove $(BUILD_DIR)"
	@echo ""
	@echo "inputs (set RIO_LOG / RIO_ULG / RIO_MOCAP in your env, or override on CLI):"
	@echo "  LOG=$(LOG)      [$(call src,LOG,RIO_LOG)]"
	@echo "  ULG=$(ULG)      [$(call src,ULG,RIO_ULG)]"
	@echo "  MOCAP=$(MOCAP)  [$(call src,MOCAP,RIO_MOCAP)]"
	@echo ""
	@echo "other variables:"
	@echo "  NAME=$(NAME)         (derived from LOG basename)"
	@echo "  OUT=$(OUT)"
	@echo "  PY=$(PY)"
	@echo ""
	@echo "replay aiding flags: edit src/config.h (shared with firmware):"
	@echo "    BARO_AIDING_ENABLED       (0/1) — master baro on/off"
	@echo "    BARO_AIDING_DIFFERENTIAL  (0/1) — 1=differential, 0=absolute"
	@echo "    RADAR_AIDING_ENABLED      (0/1) — master radar on/off"
	@echo "  then 'make build' to pick up the change."
	@echo ""
	@echo "compare flags (forwarded to compare_with_ulog.py):"
	@echo "  ARGS=$(ARGS)"
	@echo "  shortcuts:"
	@echo "    NO_VVO=1     → adds --no-vvo   (suppress live VVO overlay)"
	@echo "    TOL=0.3      → adds --tol 0.3  (association tolerance, s)"
	@echo "    SKIP=2       → adds --skip-seconds 2  (drop first 2s after VVO start)"
	@echo "    ALIGN_EKF2=1 → adds --align-to-ekf2  (origin-align onto EKF2 attitude, not identity)"
	@echo "    NO_MOUNT=1   → adds --no-mount-correction  (no q_m on ANY trace: skipped on replay, un-baked from live VVO; also honored by 'make ablation')"
	@echo "    MOCAP=path/to.bag    → adds --mocap (overlay ROS1 mocap bag)"
	@echo "    MOCAP_OFFSET=1.23    → adds --mocap-offset 1.23  (override |v|-autosync, s)"
	@echo "    MOCAP_SYNC=vvo       → adds --mocap-sync-with vvo  (replay|vvo|ekf2; default replay)"
	@echo "    MOCAP_FRAME=bld      → adds --mocap-body-frame bld  (default frd; bld applies BLD→FRD body fix)"
