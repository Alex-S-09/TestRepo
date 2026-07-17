# 📑 Git Cheat Sheet for Robot Coders

Follow these steps every time you start a new project or a new day of coding!

---

## 🏎️ 1. Get the Latest Code (The "Sync")
Before you start, make sure you have any changes your teammates made.
*   **Command:** `git pull`
*   **What it does:** Grabs the newest code from GitHub and puts it on your computer.

## 🌿 2. Start a New Project (The "Branch")
Never code directly on `master` (the main branch). Create a "branch" instead!
*   **Command:** `git checkout -b my-new-robot`
*   **What it does:** Creates a new "room" for you to work in without breaking the main code. 
*   *Tip: Name it after your robot or task (e.g., `claw-arm-fix`).*

## 💾 3. Save Your Work (The "Commit")
Do this often! It's like a save point in a video game.
1.  **Stage it:** `git add .` (Tells Git: "I want to save these files.")
2.  **Commit it:** `git commit -m "Added the claw code"` (Tells Git: "Save it now with this note.")
3.  **Push it:** `git push origin my-new-robot` (Sends your save point to the internet/GitHub.)

## 🤝 4. Finish and Merge (The "Combine")
When your code is perfect and you want it to be part of the main robot:
1.  **Go back to main:** `git checkout master`
2.  **Pull again (just in case):** `git pull`
3.  **Merge your work:** `git merge my-new-robot`
4.  **Send it home:** `git push origin master`

---

### 💡 Pro Tips for 6th-7th Graders:
*   **Commit Messages:** Make them helpful! Instead of "stuff," say "Fixed the motor direction."
*   **Errors?** If Git says "Merge Conflict," don't panic! It just means two people edited the same line. Ask a coach or teammate for help.
*   **Check where you are:** Type `git status` anytime to see which branch you are on.
