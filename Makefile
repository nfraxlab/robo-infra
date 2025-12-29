SHELL := /bin/bash

# Default for make pr sync flag
sync ?= 0

.PHONY: help install lint format format-check type typecheck test unit unitv cov clean clean-pycache sim hw ci check docs docs-serve docs-build

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
#   make pr m="feat: add new feature"
#   make pr m="fix: bug" sync=1   # optional: rebase feature branch on origin/main before pushing
pr:
ifndef m
	$(error Usage: make pr m="feat: your commit message")
endif
	@set -euo pipefail; \
	if [ -z "$(m)" ]; then echo "[pr] ERROR: Commit message cannot be empty."; exit 1; fi; \
	gh auth status >/dev/null 2>&1 || { echo "[pr] ERROR: gh CLI not authenticated. Run 'gh auth login' first."; exit 1; }; \
	git remote get-url origin >/dev/null 2>&1 || { echo "[pr] ERROR: remote 'origin' not found."; exit 1; }; \
	CURRENT_BRANCH=$$(git branch --show-current || true); \
	if [ -z "$$CURRENT_BRANCH" ]; then \
		echo "[pr] ERROR: Detached HEAD state. Checkout a branch first."; \
		exit 1; \
	fi; \
	SYNC_FLAG="$(sync)"; \
	if [ "$$SYNC_FLAG" != "1" ]; then SYNC_FLAG="0"; fi; \
	if [ "$$CURRENT_BRANCH" = "main" ]; then \
		echo "[pr] On main - creating new PR for: $(m)"; \
		TIMESTAMP=$$(date -u +%m%d%H%M); \
		MSG_NO_PREFIX=$$(echo "$(m)" | sed -E 's/^[a-zA-Z]+(\([^)]+\))?!?:[ ]*//'); \
		SLUG=$$(echo "$$MSG_NO_PREFIX" | tr '[:upper:]' '[:lower:]' | sed 's/[^a-z0-9]/-/g' | sed 's/--*/-/g' | sed 's/^-//' | sed 's/-$$//' | cut -c1-40); \
		[ -z "$$SLUG" ] && SLUG="change"; \
		BRANCH="$$SLUG-$$TIMESTAMP"; \
		git fetch origin main >/dev/null; \
		git pull --ff-only origin main || { echo "[pr] ERROR: main is not fast-forwardable. Resolve manually."; exit 1; }; \
		git checkout -b "$$BRANCH"; \
		git add -A; \
		if git diff --cached --quiet; then \
			echo "[pr] No changes to commit. Cleaning up branch."; \
			git checkout main >/dev/null; \
			git branch -D "$$BRANCH" >/dev/null; \
			exit 0; \
		fi; \
		git commit -m "$(m)"; \
		git push --set-upstream origin "$$BRANCH"; \
		if gh pr view "$$BRANCH" >/dev/null 2>&1; then \
			echo "[pr] PR already exists: $$(gh pr view "$$BRANCH" --json url -q .url)"; \
		else \
			gh pr create --title "$(m)" --body "$(m)" --base main --head "$$BRANCH"; \
		fi; \
		git checkout main >/dev/null; \
		echo "[pr] Done!"; \
	else \
		echo "[pr] On branch $$CURRENT_BRANCH - updating/creating PR"; \
		if [ "$$SYNC_FLAG" = "1" ]; then \
			echo "[pr] Sync enabled - rebasing $$CURRENT_BRANCH on origin/main"; \
			git fetch origin main >/dev/null; \
			git rebase origin/main || { echo "[pr] ERROR: Rebase failed. Run 'git rebase --abort' and resolve manually."; exit 1; }; \
		fi; \
		git add -A; \
		COMMITTED=0; \
		if git diff --cached --quiet; then \
			echo "[pr] No changes to commit"; \
		else \
			git commit -m "$(m)"; \
			COMMITTED=1; \
		fi; \
		if [ "$$SYNC_FLAG" = "1" ]; then \
			git push --force-with-lease origin "$$CURRENT_BRANCH"; \
		elif [ "$$COMMITTED" = "1" ] || ! git rev-parse --verify origin/"$$CURRENT_BRANCH" >/dev/null 2>&1; then \
			git push -u origin "$$CURRENT_BRANCH"; \
		fi; \
		if gh pr view "$$CURRENT_BRANCH" >/dev/null 2>&1; then \
			echo "[pr] PR exists: $$(gh pr view "$$CURRENT_BRANCH" --json url -q .url)"; \
		else \
			gh pr create --title "$(m)" --body "$(m)" --base main --head "$$CURRENT_BRANCH"; \
			echo "[pr] PR created."; \
		fi; \
	fi

# Usage: make commit m="feat: add new feature"
# Just commits with proper message (for when you want to batch commits before PR)
commit:
ifndef m
	$(error Usage: make commit m="feat: your commit message")
endif
	git add -A
	git commit -m "$(m)"
