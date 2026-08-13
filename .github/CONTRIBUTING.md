# Contributing Guidelines

Welcome to the team! To maintain a highly reliable product and a clean Git history, all collaborators must follow these exact repository workflow rules.

---

## 1. Branching Strategy

You are strictly blocked from pushing code directly to the `main` branch. 

* **Rule:** Always create a new branch from the latest `main` branch before you start making changes.
* **Naming Convention:** Use clear names for your branches so we know what you are doing:
  * For features: `feature/your-feature-name` (e.g., `feature/login-page`)
  * For bug fixes: `bugfix/your-fix-name` (e.g., `bugfix/broken-button`)

---

## 2. Pull Request & Merging Rules

Once your work is finished on your side branch, you must open a Pull Request (PR) on GitHub.

* 📥 **Merge Commits:** We strongly recommend using the default **Merge Commit** option on GitHub. Do not squash your commits and do not rebase, unless absolutely necessary. We want to preserve your full step-by-step history log.
* 👀 **Required Approvals:** You must receive approval from at least one member of the team before merging your pull request.

---

## 3. If you accidentally commited to main:

If you forgot to switch branches and accidentally hit the "Commit" button in GitHub Desktop while on `main`, **do not panic**. Your local push will be blocked, and your code will not break the server.

Please immediately read our **[Troubleshooting Guide (TROUBLESHOOTING.md)](TROUBLESHOOTING.md)** which will guide you through moving your local commit to a safe feature branch without losing any of your hard work.

---

Thank you for following these rules and maintaining a clean, organized repository!
