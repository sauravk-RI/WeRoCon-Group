# Contributing to WeRoCon-Group

Thanks for helping build the lab's knowledge base. These rules keep the repository easy to follow
for the next student.

## Workflow

1. **Never commit straight to `main`.** It is protected, so every change goes through a pull request (PR).
2. Create a branch named after your work, for example `imu/mpu9250-calibration` or `motor/ankle-tracking`.
3. Keep each PR focused on one topic and describe what you tested on hardware.
4. The PI or a maintainer reviews and merges it. Branches are deleted automatically after merging.

New to Git? Follow the
[step-by-step contribution guide](Readme_How_To_Do_Open_Source_Contribution_To_This_Repository_For_New_Github_Users.md).

## Where things go

| You are adding… | Put it in |
|-----------------|-----------|
| A script | `<Subsystem>/Codes/` |
| A setup guide or explanation | `<Subsystem>/Documentation/` |
| Images used by a guide | Next to the guide, or in an `ImageFiles/` folder beside it |
| Sample logs and result plots | `<Subsystem>/Results/` |
| A new subsystem | A new top-level folder with its own `Readme.md`, plus a row in the root README's repository map |

## Writing conventions

- **File names:** use underscores, not spaces, for example `Knee_Trajectory_Tracking.md`. Avoid copies like `file (1).md`.
- **Links and images:** always use forward slashes (`ImageFiles/plot.png`, not `ImageFiles\plot.png`). Backslashes break on GitHub.
- **Use descriptive image names,** such as `before_calibration_mpu6050.png`, not screenshot timestamps, when practical.
- **Start every guide** with what it achieves, the hardware needed and the prerequisites.
- **Credit sources** (datasheets, papers, libraries) with links, and link to vendor datasheets instead of uploading them.
- **Commit messages** should say what changed and why: "Fix yaw drift in MPU9250 Kalman filter" rather than "Update file".

## Do not commit

- Passwords, Wi-Fi credentials, IITJ login IDs or API keys. Use placeholders like `YOUR_PASSWORD`.
- Unpublished data or results the PI hasn't cleared for public release.
- Large raw datasets or videos. Upload them to the lab drive and link to them instead.
