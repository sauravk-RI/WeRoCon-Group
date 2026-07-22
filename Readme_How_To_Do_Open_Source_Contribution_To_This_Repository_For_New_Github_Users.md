# Open Source Contribution Workflow on GitHub
*A Beginner-Friendly Step-by-Step Guide*

---

# Table of Contents

1. Introduction
2. Understanding the Workflow
3. Prerequisites
4. Step 1 – Fork the Repository
5. Step 2 – Clone Your Fork
6. Step 3 – Configure the Original Repository as Upstream
7. Step 4 – Create a New Branch
8. Step 5 – Make Your Changes
9. Step 6 – Commit Your Changes
10. Step 7 – Push the Branch to Your Fork
11. Step 8 – Create a Pull Request
12. Step 9 – Respond to Review Comments (if any)
13. Step 10 – After Your Pull Request is Merged
14. Step 11 – Delete the Feature Branch
15. Recommended Workflow for Future Contributions
16. Common Git Commands Cheat Sheet
17. Frequently Asked Questions

---

# 1. Introduction

One of the biggest advantages of GitHub is that anyone can contribute to open-source projects—even if they are not the owner of the repository.

The standard workflow used by almost every open-source project is:

```
Original Repository
        │
      Fork
        │
Clone Your Fork
        │
Create a New Branch
        │
Make Changes
        │
Commit Changes
        │
Push Branch
        │
Create Pull Request
        │
Repository Owner Reviews
        │
Merge Pull Request
        │
Sync Your Fork
```

This guide explains every step in detail.

---

# 2. Understanding the Workflow

Suppose there is a repository owned by a project maintainer.

```
Original Repository

johnsmith/MyProject
```

You do **not** have permission to directly modify it.

Instead, GitHub creates your own personal copy of the repository.

```
Your GitHub Account

yourname/MyProject
```

This personal copy is called a **Fork**.

You make changes inside your own fork and then request the maintainer to include those changes by creating a **Pull Request (PR)**.

---

# 3. Prerequisites

Before starting, ensure that you have:

- A GitHub account
- Git installed on your computer
- Basic familiarity with the terminal or command prompt
- Permission to contribute to the repository (public repositories generally allow Pull Requests)

Check Git installation:

```bash
git --version
```

---

# 4. Step 1 – Fork the Repository

Open the repository you want to contribute to.

Example:

```
https://github.com/johnsmith/MyProject
```

Click the **Fork** button in the top-right corner.

GitHub creates

```
yourname/MyProject
```

This repository belongs entirely to you.

You now have complete freedom to edit it.

---

# 5. Step 2 – Clone Your Fork

Copy the URL of **your fork**, not the original repository.

Example:

```
https://github.com/yourname/MyProject.git
```

Clone it:

```bash
git clone https://github.com/yourname/MyProject.git
```

Move into the project folder:

```bash
cd MyProject
```

---

# 6. Step 3 – Configure the Original Repository as Upstream

Although optional, this is highly recommended.

Your local repository currently knows only about your fork.

Verify:

```bash
git remote -v
```

Output:

```
origin
https://github.com/yourname/MyProject.git
```

Now add the original repository as an additional remote called **upstream**.

```bash
git remote add upstream https://github.com/johnsmith/MyProject.git
```

Verify again:

```bash
git remote -v
```

Output:

```
origin
https://github.com/yourname/MyProject.git

upstream
https://github.com/johnsmith/MyProject.git
```

Now your local repository knows about both repositories.

---

# 7. Step 4 – Create a New Branch

Never make changes directly on the `main` branch.

Always create a separate branch.

Example:

```bash
git checkout -b add-installation-guide
```

You are now working on:

```
add-installation-guide
```

instead of

```
main
```

This keeps your work isolated.

---

# 8. Step 5 – Make Your Changes

Edit files using your preferred editor.

Examples:

- Fix documentation
- Correct spelling mistakes
- Add new tutorials
- Improve code
- Fix bugs
- Add features

Test your changes before committing them.

---

# 9. Step 6 – Commit Your Changes

Check modified files:

```bash
git status
```

Stage them:

```bash
git add .
```

or stage specific files:

```bash
git add README.md
```

Commit:

```bash
git commit -m "Add Raspberry Pi 5 beginner tutorial"
```

A commit is simply a saved checkpoint of your work.

---

# 10. Step 7 – Push the Branch to Your Fork

Push your new branch to GitHub.

```bash
git push origin add-installation-guide
```

Your fork now contains

```
main

add-installation-guide
```

---

# 11. Step 8 – Create a Pull Request

Open your fork on GitHub.

```
yourname/MyProject
```

GitHub usually displays

```
Compare & Pull Request
```

Click it.

If it does not appear:

```
Pull Requests

↓

New Pull Request
```

Ensure the repositories are correct.

The Pull Request should look like:

```
Base Repository

johnsmith/MyProject

Base Branch

main

↓

Head Repository

yourname/MyProject

Compare Branch

add-installation-guide
```

This means:

> "Please merge my branch into your main branch."

Fill in:

- Title
- Description
- Summary of changes

Click

```
Create Pull Request
```

Done!

---

# 12. Step 9 – Respond to Review Comments (if any)

The repository owner may

- Approve your changes
- Request modifications
- Ask questions
- Suggest improvements

If changes are requested:

Edit your files locally.

Commit again.

```bash
git add .
git commit -m "Address review comments"
```

Push again.

```bash
git push origin add-installation-guide
```

The Pull Request updates automatically.

No need to create a new Pull Request.

---

# 13. Step 10 – After Your Pull Request is Merged

Suppose the maintainer clicks

```
Merge Pull Request
```

The original repository now contains your contribution.

However...

**Your fork is NOT automatically updated.**

Many beginners assume this happens automatically.

It does not.

---

## Updating Your Fork

### Method 1 (Recommended)

Open your fork on GitHub.

Click

```
Sync Fork
```

↓

```
Update Branch
```

Your fork's `main` branch now matches the original repository.

---

### Method 2 (Using Git)

Fetch changes from the original repository.

```bash
git fetch upstream
```

Switch to the main branch.

```bash
git checkout main
```

Merge upstream changes.

```bash
git merge upstream/main
```

Push the updated main branch to your fork.

```bash
git push origin main
```

Your fork is now synchronized.

---

# 14. Step 11 – Delete the Feature Branch

After the Pull Request has been merged, the feature branch is no longer needed.

Delete it locally.

```bash
git branch -d add-installation-guide
```

Delete it from GitHub.

```bash
git push origin --delete add-installation-guide
```

Your repository stays clean.

---

# 15. Recommended Workflow for Future Contributions

Every new contribution should follow the same pattern.

```
Sync Fork

↓

Update Local Main

↓

Create New Branch

↓

Make Changes

↓

Commit

↓

Push

↓

Create Pull Request

↓

Owner Reviews

↓

Merge

↓

Sync Fork Again

↓

Delete Branch

↓

Repeat
```

Never continue adding unrelated work to an old feature branch.

Create a fresh branch for every independent contribution.

---

# 16. Common Git Commands Cheat Sheet

## Clone a repository

```bash
git clone <repository-url>
```

---

## Move into the project

```bash
cd project-name
```

---

## Create a new branch

```bash
git checkout -b feature-name
```

---

## View current branch

```bash
git branch
```

---

## Check modified files

```bash
git status
```

---

## Stage all changes

```bash
git add .
```

---

## Commit changes

```bash
git commit -m "Meaningful commit message"
```

---

## Push branch

```bash
git push origin feature-name
```

---

## Fetch latest upstream changes

```bash
git fetch upstream
```

---

## Merge upstream changes into main

```bash
git checkout main
git merge upstream/main
```

---

## Push updated main

```bash
git push origin main
```

---

## Delete local branch

```bash
git branch -d feature-name
```

---

## Delete remote branch

```bash
git push origin --delete feature-name
```

---

# 17. Frequently Asked Questions

## Why should I create a separate branch?

A separate branch isolates your work from the stable `main` branch, making it easier to review, test, and manage changes.

---

## Can I create multiple Pull Requests?

Yes.

Each feature or bug fix should have its own branch and its own Pull Request.

---

## What happens if my Pull Request is rejected?

Nothing changes in your fork.

Your branch remains available, and you can continue improving it or use it for your own work.

---

## Can I continue working after opening a Pull Request?

Yes.

Simply commit new changes to the same branch and push them.

The existing Pull Request updates automatically.

---

## Does merging automatically update my fork?

No.

You must manually synchronize your fork using either:

- **GitHub's "Sync fork" button**, or
- **Git commands** (`fetch`, `merge`, and `push`).

---

# Conclusion

The standard GitHub open-source contribution workflow is:

```
Fork Repository
        │
Clone Your Fork
        │
Configure Upstream
        │
Create Feature Branch
        │
Make Changes
        │
Test Changes
        │
Commit
        │
Push Branch
        │
Create Pull Request
        │
Owner Reviews
        │
Merge Pull Request
        │
Sync Your Fork
        │
Delete Feature Branch
        │
Repeat
```

Following this workflow ensures that:
- The original repository remains stable.
- Your contributions are isolated and easy to review.
- Your fork stays synchronized with the latest changes.
- Each contribution is clean, organized, and easy to maintain.