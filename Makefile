SHELL := /bin/bash

.PHONY: help install lint format typecheck test unit unitv cov clean clean-pycache sim

help: ## Show available commands
	@echo "Available commands:"
	@echo ""
	@echo "Setup:"
	@echo "  install           Install package with dev dependencies"
	@echo ""
	@echo "Code Quality:"
	@echo "  lint              Run ruff linter"
	@echo "  format            Format code with ruff"
	@echo "  typecheck         Run mypy type checker"
	@echo "  check             Run all checks (lint + typecheck)"
	@echo ""
	@echo "Testing:"
	@echo "  test              Run all tests"
	@echo "  unit              Run unit tests (quiet)"
	@echo "  unitv             Run unit tests (verbose)"
	@echo "  cov               Run tests with coverage report"
	@echo "  sim               Run simulation tests only"
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

typecheck:
	@echo "[typecheck] Running mypy..."
	@poetry run mypy src

check: lint typecheck
	@echo "[check] All checks passed"

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
