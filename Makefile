SHELL := /bin/bash

# Defaults for make pr flags
sync  ?= 0
new   ?= 0
draft ?= 0
b     ?=
base  ?=

.PHONY: help install lint format format-check type typecheck test unit unitv cov clean clean-pycache sim hw ci check docs docs-serve docs-build report

help: ## Show available commands
	@echo "Available commands:"
	@echo ""
	@echo "Setup:"
	@echo "  install           Install package with dev dependencies"
	@echo ""
	@echo "Code Quality:"
	@echo "  lint              Run ruff linter"
	@echo "  format            Format code with ruff"
	@echo "  format-check      Check formatting (no changes)"
	@echo "  type              Run mypy type checker"
	@echo "  typecheck         Alias for type"
	@echo "  check             Run all checks (lint + type)"
	@echo "  ci                Run checks + tests (for CI pipelines)"
	@echo "  report            Production readiness analysis report"
	@echo ""
	@echo "Testing:"
	@echo "  test              Run all tests"
	@echo "  unit              Run unit tests (quiet)"
	@echo "  unitv             Run unit tests (verbose)"
	@echo "  cov               Run tests with coverage report"
	@echo "  sim               Run simulation tests only"
	@echo "  hw                Run hardware tests (requires hardware)"
	@echo ""
	@echo "Documentation:"
	@echo "  docs              Serve docs locally with live reload"
	@echo "  docs-build        Build docs for production"
	@echo ""
	@echo "Cleanup:"
	@echo "  clean             Remove caches, build artifacts"
	@echo "  clean-pycache     Remove only __pycache__ directories"
	@echo ""

# --- Setup ---
install:
	@echo "[install] Installing package with dev dependencies..."
	poetry install --no-interaction

# --- Code Quality ---
lint:
	@echo "[lint] Running ruff..."
	@poetry run ruff check src tests

format:
	@echo "[format] Formatting code..."
	@poetry run ruff format src tests
	@poetry run ruff check --fix src tests

format-check:
	@echo "[format-check] Checking formatting (no changes)..."
	@poetry run ruff format --check src tests

type:
	@echo "[type] Running mypy..."
	@poetry run mypy src

typecheck: type

check: lint type
	@echo "[check] All checks passed"

ci: check test
	@echo "[ci] All checks + tests passed"

# --- Production Readiness Report ---
# Minimum coverage threshold (override with: make report COV_MIN=70)
# Strict mode for CI (override with: make report STRICT=1)
# Report mode: full (default) or ci (skip lint/mypy/pytest, assume CI already ran them)
COV_MIN ?= 60
STRICT ?= 0
REPORT_MODE ?= full

.PHONY: report

report: ## Production readiness gate (CI-friendly with exit codes)
	@set -euo pipefail; \
	echo ""; \
	echo "╔══════════════════════════════════════════════════════════════════════════════╗"; \
	echo "║                    🚀 PRODUCTION READINESS GATE                              ║"; \
	echo "║                          robo-infra                                          ║"; \
	echo "╚══════════════════════════════════════════════════════════════════════════════╝"; \
	echo ""; \
	if [ "$(REPORT_MODE)" = "ci" ]; then \
		if [ "$${CI:-}" != "true" ]; then \
			echo "❌ ERROR: REPORT_MODE=ci requires CI=true environment variable"; \
			echo "   This mode should only be used in GitHub Actions, not locally."; \
			echo "   Run 'make report' instead for full local checks."; \
			exit 1; \
		fi; \
		: "$${LINT_PASSED:?REPORT_MODE=ci requires LINT_PASSED=1 from upstream job}"; \
		: "$${TYPE_PASSED:?REPORT_MODE=ci requires TYPE_PASSED=1 from upstream job}"; \
		: "$${TESTS_PASSED:?REPORT_MODE=ci requires TESTS_PASSED=1 from upstream job}"; \
	fi; \
	VERSION=$$(poetry version -s 2>/dev/null || echo "unknown"); \
	echo "📦 Package Version: $$VERSION"; \
	echo "📋 Coverage Minimum: $(COV_MIN)%"; \
	if [ "$(STRICT)" = "1" ]; then echo "🔒 Strict Mode: ON (fails if score < 9/11)"; fi; \
	if [ "$(REPORT_MODE)" = "ci" ]; then echo "⚡ CI Mode: ON (skipping lint/mypy/pytest)"; fi; \
	echo ""; \
	\
	echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; \
	echo "🔍 RUNNING ALL CHECKS..."; \
	echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; \
	echo ""; \
	\
	SCORE=0; \
	CRITICAL_FAIL=0; \
	\
	echo "① Linting (ruff)..."; \
	if [ "$(REPORT_MODE)" = "ci" ]; then \
		echo "   ⏭️  SKIP (CI mode - already ran in CI)"; \
		LINT_OK=1; SCORE=$$((SCORE + 1)); \
	else \
		set +e; poetry run ruff check src tests >/dev/null 2>&1; LINT_EXIT=$$?; set -e; \
		if [ "$$LINT_EXIT" -eq 0 ]; then \
			echo "   ✅ PASS (1 pt)"; \
			LINT_OK=1; SCORE=$$((SCORE + 1)); \
		else \
			echo "   ❌ FAIL - linting errors found:"; \
			poetry run ruff check src tests 2>&1 | head -20; \
			LINT_OK=0; \
		fi; \
	fi; \
	echo ""; \
	\
	echo "② Type checking (mypy)..."; \
	if [ "$(REPORT_MODE)" = "ci" ]; then \
		echo "   ⏭️  SKIP (CI mode - already ran in CI)"; \
		TYPE_OK=1; SCORE=$$((SCORE + 1)); \
	else \
		set +e; poetry run mypy src >/dev/null 2>&1; MYPY_EXIT=$$?; set -e; \
		if [ "$$MYPY_EXIT" -eq 0 ]; then \
			echo "   ✅ PASS (1 pt)"; \
			TYPE_OK=1; SCORE=$$((SCORE + 1)); \
		else \
			echo "   ❌ FAIL - type errors found:"; \
			poetry run mypy src 2>&1 | head -20; \
			TYPE_OK=0; \
		fi; \
	fi; \
	echo ""; \
	\
	echo "③ Tests + Coverage (min $(COV_MIN)%)..."; \
	if [ "$(REPORT_MODE)" = "ci" ]; then \
		echo "   ⏭️  SKIP (CI mode - already ran in CI)"; \
		TEST_OK=1; COV_OK=1; SCORE=$$((SCORE + 4)); \
	else \
		set +e; COV_OUTPUT=$$(poetry run pytest --cov=src --cov-report=term-missing -q tests/unit 2>&1); TEST_EXIT=$$?; set -e; \
		COV_PCT=$$(echo "$$COV_OUTPUT" | awk '/^TOTAL/ {for(i=1;i<=NF;i++) if($$i ~ /%$$/) {gsub(/%/,"",$$i); print $$i; exit}}'); \
		if [ -z "$$COV_PCT" ]; then \
			if echo "$$COV_OUTPUT" | grep -qE "unrecognized arguments: --cov|pytest_cov|No module named.*pytest_cov"; then \
				echo "   ❌ FAIL - pytest-cov not installed (poetry add --group dev pytest-cov)"; \
			else \
				echo "   ❌ FAIL - tests failed or no coverage data"; \
				echo "$$COV_OUTPUT" | tail -10; \
			fi; \
			TEST_OK=0; COV_OK=0; CRITICAL_FAIL=1; \
		elif [ "$$TEST_EXIT" -ne 0 ]; then \
			echo "   ❌ FAIL - tests failed"; \
			echo "$$COV_OUTPUT" | tail -10; \
			TEST_OK=0; COV_OK=0; CRITICAL_FAIL=1; \
		elif [ "$$COV_PCT" -lt $(COV_MIN) ]; then \
			echo "   ❌ FAIL - tests passed but $${COV_PCT}% coverage below $(COV_MIN)%"; \
			TEST_OK=1; COV_OK=0; SCORE=$$((SCORE + 2)); CRITICAL_FAIL=1; \
		else \
			echo "   ✅ PASS - $${COV_PCT}% coverage (4 pts: 2 tests + 2 coverage)"; \
			TEST_OK=1; COV_OK=1; SCORE=$$((SCORE + 4)); \
		fi; \
	fi; \
	echo ""; \
	\
	echo "④ Security: Vulnerability scan (pip-audit)..."; \
	if poetry run pip-audit --version >/dev/null 2>&1; then \
		set +e; poetry run pip-audit >/dev/null 2>&1; AUDIT_EXIT=$$?; set -e; \
		if [ "$$AUDIT_EXIT" -eq 0 ]; then \
			echo "   ✅ PASS - no known vulnerabilities (2 pts)"; \
			VULN_OK=1; SCORE=$$((SCORE + 2)); \
		else \
			echo "   ❌ FAIL - vulnerabilities found"; \
			poetry run pip-audit 2>&1 | head -15; \
			VULN_OK=0; CRITICAL_FAIL=1; \
		fi; \
	else \
		if [ "$(STRICT)" = "1" ]; then \
			echo "   ❌ FAIL - pip-audit required in STRICT mode (poetry add --group dev pip-audit)"; \
			VULN_OK=0; CRITICAL_FAIL=1; \
		else \
			echo "   ⚠️  SKIP - pip-audit not installed (0 pts)"; \
			VULN_OK=0; \
		fi; \
	fi; \
	echo ""; \
	\
	echo "⑤ Package build + verification..."; \
	rm -rf dist/; \
	if poetry build -q 2>/dev/null; then \
		if poetry run twine --version >/dev/null 2>&1 && poetry run twine check dist/* >/dev/null 2>&1; then \
			echo "   ✅ PASS - package builds and passes twine check (2 pts)"; \
			BUILD_OK=1; SCORE=$$((SCORE + 2)); \
		elif poetry run python -m zipfile -t dist/*.whl >/dev/null 2>&1; then \
			echo "   ✅ PASS - package builds, wheel is valid (2 pts)"; \
			BUILD_OK=1; SCORE=$$((SCORE + 2)); \
		else \
			echo "   ✅ PASS - package builds (2 pts)"; \
			BUILD_OK=1; SCORE=$$((SCORE + 2)); \
		fi; \
	else \
		echo "   ❌ FAIL - package build failed"; \
		BUILD_OK=0; CRITICAL_FAIL=1; \
	fi; \
	echo ""; \
	\
	echo "⑥ Documentation..."; \
	DOC_SCORE=0; \
	[ -f README.md ] && DOC_SCORE=$$((DOC_SCORE + 1)); \
	[ -f CHANGELOG.md ] && DOC_SCORE=$$((DOC_SCORE + 1)); \
	[ -d docs ] && DOC_SCORE=$$((DOC_SCORE + 1)); \
	if [ "$$DOC_SCORE" -ge 2 ]; then \
		echo "   ✅ PASS - core docs present (1 pt)"; \
		DOCS_OK=1; SCORE=$$((SCORE + 1)); \
	else \
		echo "   ❌ FAIL - missing README.md, CHANGELOG.md, or docs/"; \
		DOCS_OK=0; \
	fi; \
	echo ""; \
	\
	echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; \
	echo "📋 RESULTS"; \
	echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; \
	echo ""; \
	echo "  Component          Weight    Status"; \
	echo "  ─────────────────────────────────────"; \
	[ "$$LINT_OK" = "1" ] && echo "  Linting            1 pt      ✅" || echo "  Linting            1 pt      ❌"; \
	[ "$$TYPE_OK" = "1" ] && echo "  Type checking      1 pt      ✅" || echo "  Type checking      1 pt      ❌"; \
	[ "$$TEST_OK" = "1" ] && echo "  Tests pass         2 pts     ✅" || echo "  Tests pass         2 pts     ❌ CRITICAL"; \
	[ "$$COV_OK" = "1" ] && echo "  Coverage ≥$(COV_MIN)%     2 pts     ✅" || echo "  Coverage ≥$(COV_MIN)%     2 pts     ❌ CRITICAL"; \
	if [ "$$VULN_OK" = "1" ]; then echo "  No vulnerabilities 2 pts     ✅"; \
	elif [ "$(STRICT)" = "1" ]; then echo "  No vulnerabilities 2 pts     ❌ CRITICAL"; \
	else echo "  No vulnerabilities 2 pts     ⚠️  SKIP"; fi; \
	[ "$$BUILD_OK" = "1" ] && echo "  Package builds     2 pts     ✅" || echo "  Package builds     2 pts     ❌ CRITICAL"; \
	[ "$$DOCS_OK" = "1" ] && echo "  Documentation      1 pt      ✅" || echo "  Documentation      1 pt      ❌"; \
	echo "  ─────────────────────────────────────"; \
	echo "  TOTAL              11 pts    $$SCORE"; \
	echo ""; \
	\
	PERCENT=$$((SCORE * 100 / 11)); \
	echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; \
	if [ "$$CRITICAL_FAIL" = "1" ]; then \
		echo ""; \
		echo "  ❌ NOT READY FOR PRODUCTION ($$PERCENT% - $$SCORE/11 pts)"; \
		echo ""; \
		echo "  Critical failures detected. Fix before release:"; \
		[ "$$TEST_OK" = "0" ] && echo "    • Tests must pass"; \
		[ "$$COV_OK" = "0" ] && echo "    • Coverage must be ≥$(COV_MIN)%"; \
		[ "$$VULN_OK" = "0" ] && [ "$(STRICT)" = "1" ] && echo "    • Vulnerabilities must be resolved"; \
		[ "$$BUILD_OK" = "0" ] && echo "    • Package must build successfully"; \
		echo ""; \
		echo "╚══════════════════════════════════════════════════════════════════════════════╝"; \
		exit 1; \
	elif [ "$(STRICT)" = "1" ] && [ "$$SCORE" -lt 9 ]; then \
		echo ""; \
		echo "  ❌ STRICT MODE: Score $$SCORE/11 is below 9/11 threshold"; \
		echo ""; \
		echo "╚══════════════════════════════════════════════════════════════════════════════╝"; \
		exit 1; \
	elif [ "$$SCORE" -ge 9 ]; then \
		echo ""; \
		echo "  ✅ READY FOR PRODUCTION ($$PERCENT% - $$SCORE/11 pts)"; \
		echo ""; \
		echo "╚══════════════════════════════════════════════════════════════════════════════╝"; \
	else \
		echo ""; \
		echo "  ⚠️  NEEDS WORK ($$PERCENT% - $$SCORE/11 pts)"; \
		echo ""; \
		echo "  No critical failures, but score below 9/11."; \
		echo "  Use STRICT=1 to enforce in CI."; \
		echo ""; \
		echo "╚══════════════════════════════════════════════════════════════════════════════╝"; \
	fi

# --- Testing ---
test:
	@echo "[test] Running all tests..."
	@poetry run pytest

unit:
	@echo "[unit] Running unit tests (quiet)..."
	@poetry run pytest -q tests/unit

unitv:
	@echo "[unit] Running unit tests (verbose)..."
	@poetry run pytest -vv tests/unit

cov:
	@echo "[cov] Running tests with coverage..."
	@poetry run pytest --cov=robo_infra --cov-report=term-missing --cov-report=html tests/
	@echo "[cov] HTML report: htmlcov/index.html"

sim:
	@echo "[sim] Running simulation tests..."
	@poetry run pytest tests/integration -m "not hardware"

# --- Hardware Testing (requires real hardware) ---
hw:
	@echo "[hw] Running hardware tests (requires connected hardware)..."
	@poetry run pytest tests/integration -m "hardware"

# --- Cleanup ---
clean: clean-pycache
	@echo "[clean] Removing build artifacts..."
	@rm -rf dist/ build/ *.egg-info .pytest_cache .mypy_cache .ruff_cache htmlcov .coverage
	@find . -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true

clean-pycache:
	@echo "[clean] Removing __pycache__ directories..."
	@find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	@find . -type f -name "*.pyc" -delete 2>/dev/null || true

# --- Development Helpers ---
dev:
	@echo "[dev] Starting development environment..."
	@poetry shell

publish:
	@echo "[publish] Building and publishing to PyPI..."
	@poetry build
	@poetry publish

# --- CLI ---
cli:
	@echo "[cli] Running robo-infra CLI..."
	@poetry run robo-infra $(ARGS)

# --- Documentation ---
.PHONY: docs docs-serve docs-build

docs: docs-serve ## Alias for docs-serve

docs-serve: ## Serve documentation locally with live reload
	@echo "[docs] Starting documentation server..."
	poetry run mkdocs serve

docs-build: ## Build documentation for production
	@echo "[docs] Building documentation..."
	poetry run mkdocs build

# --- Git/PR Automation ---
.PHONY: pr commit

# Usage:
#   make pr m="feat: add new feature"        # Create PR from main or update existing PR
#   make pr m="fix: bug" sync=1              # Rebase feature branch on default branch before pushing
#   make pr m="wip" FORCE=1                  # Override conventional commit check
#   make pr m="feat: new" new=1              # Create new PR from feature branch (new branch from HEAD)
#   make pr m="feat: add" b="feat/my-branch" # Use explicit branch name
#   make pr m="feat: wip" draft=1            # Create PR as draft
#   make pr m="fix: hotfix" base=release     # Target a different base branch
pr:
ifndef m
	$(error Usage: make pr m="feat: your commit message")
endif
	@./scripts/pr.sh "$(m)" "$(sync)" "$(new)" "$(b)" "$${FORCE:-0}" "$(draft)" "$(base)"

# Usage: make commit m="feat: add new feature"
# Just commits with proper message (for when you want to batch commits before PR)
commit:
ifndef m
	$(error Usage: make commit m="feat: your commit message")
endif
	@git add -A && git commit -m "$(m)"
