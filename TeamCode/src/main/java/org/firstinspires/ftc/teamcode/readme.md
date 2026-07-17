# 🤖 TeamCode: Our Robot's Brain!

Welcome to the **TeamCode** folder! This is where we write the instructions that tell our robot how to move, think, and win!

## 🚀 Our First Code: `FirstCode_SB.java`

This is our very first **TeleOp** program. "TeleOp" is short for "Tele-Operated," which just means you get to drive the robot using a game controller!

### 🕹️ How to Drive
We use the **Gamepad 1** (the first controller) to move:
*   **Move Forward & Backward:** Push the **Left Stick** up and down.
*   **Slide Sideways (Strafe):** Push the **Left Stick** left and right. Our robot has special wheels that let it slide like a crab! 🦀
*   **Turn Around:** Push the **Right Stick** left and right.

### ⚙️ How the Code Works
Inside `FirstCode_SB.java`, we do a few important things:

1.  **Meet the Motors:** We tell the robot about its 4 motors: `frontLeft`, `backLeft`, `frontRight`, and `backRight`.
2.  **Reverse Gear:** Since the motors on the left side are facing the opposite way, we tell the code to "reverse" them so "forward" means the same thing for every wheel.
3.  **The Calculation:** We take your joystick movements and mix them together!
    *   `Drive + Strafe + Turn` = How much power each wheel gets.
4.  **The "Max Power" Check:** If the math says a motor should go at 150% power (which is impossible!), the code "shrinks" all the powers down so the robot still moves exactly how you want it to, just at 100% speed.

---

## 🛠️ How to Add More Code
If you want to try something new:
1.  **Copy a Sample:** Look in the `FtcRobotController` folder for "Samples."
2.  **Paste it here:** Put it in this `org.firstinspires.ftc.teamcode` folder.
3.  **Give it a Name:** Start with a capital letter (like `MyCoolRobot.java`).
4.  **Remove @Disabled:** Delete the `@Disabled` line at the top so it shows up on the Driver Station phone!

Have fun building and coding! 🏎️💨
