# Lesson 6: Frequently Used Raspberry Pi Terminal Commands

@FirstAuthor: Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, July 2026. <br>
@Disclaimer: This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

---

# Why Learn Terminal Commands?

Although Raspberry Pi OS provides a graphical desktop environment, most Raspberry Pi development—especially in robotics, embedded systems, Linux administration, and ROS 2—is performed using the terminal.

The terminal allows you to:

- Execute programs
- Install software
- Configure the operating system
- Edit configuration files
- Monitor system resources
- Connect to remote computers using SSH
- Debug software
- Manage Wi-Fi connections
- Control hardware

As you continue working with your Raspberry Pi, you will gradually realize that using the terminal is often much faster than navigating through graphical menus.

This lesson serves as a **quick reference guide** for the terminal commands that Raspberry Pi users frequently use in their day-to-day work.

---

# Understanding the Linux Terminal

The Linux terminal is a command-line interface (CLI) that allows you to communicate directly with the operating system.

Instead of clicking buttons with a mouse, you type commands using your keyboard.

For example,

```bash
date
```

Output:

```text
Thu Jul 24 12:31:18 IST 2026
```

Here,

- `date` is the command.
- The operating system executes it.
- The result is displayed on the screen.

---

# Understanding the Command Prompt

When you open the terminal on your Raspberry Pi, you will see something similar to:

```bash
pi@raspberrypi:~ $
```

Each part has a meaning.

| Part | Meaning |
|------|----------|
| `pi` | Current username |
| `raspberrypi` | Hostname of the Raspberry Pi |
| `~` | Current directory (Home directory) |
| `$` | Normal user prompt |

If you become the root user using `sudo -i`, the prompt changes to:

```bash
root@raspberrypi:~ #
```

Notice that the `$` changes to `#`, indicating that you now have administrator privileges.

> 💡 **Note:** Most Raspberry Pi users should avoid working as the root user unless absolutely necessary.

---

# General Command Syntax

Most Linux commands follow the general format:

```text
command [options] [arguments]
```

Example:

```bash
ls -l Documents
```

Here,

| Component | Meaning |
|------------|----------|
| `ls` | Command |
| `-l` | Option |
| `Documents` | Argument |

---

# Getting Help

Whenever you are unsure how a command works, you can access its built-in manual.

Example:

```bash
man ls
```

or

```bash
ls --help
```

The `man` command opens the Linux manual page for the specified command.

Press **Q** to exit the manual.

> 💡 **Tip:** Learning to use the `man` command is one of the best ways to become comfortable with Linux.

---

# Terminal Navigation Commands

---

## Display the Current Working Directory

Command:

```bash
pwd
```

Example Output:

```text
/home/pi
```

The `pwd` command stands for **Print Working Directory**.

It displays the directory that you are currently working in.

This is one of the most commonly used Linux commands.

---

## List Files and Directories

Command:

```bash
ls
```

Example Output:

```text
Desktop
Documents
Downloads
Pictures
Music
```

The `ls` command displays all files and folders in the current directory.

---

## Display a Detailed List

Command:

```bash
ls -l
```

Example Output:

```text
drwxr-xr-x 2 pi pi 4096 Jul 24 Documents
-rw-r--r-- 1 pi pi 1200 Jul 24 notes.txt
```

This displays:

- File permissions
- Owner
- Group
- File size
- Date modified
- File name

---

## Display Hidden Files

Command:

```bash
ls -la
```

Hidden files in Linux begin with a dot (`.`).

Example:

```text
.bashrc
.profile
.config
```

These files usually store system or application configuration settings.

---

## Change Directory

Command:

```bash
cd folder_name
```

Example:

```bash
cd Documents
```

Moves into the Documents folder.

---

## Go Back One Directory

Command:

```bash
cd ..
```

Example:

Current directory:

```text
/home/pi/Documents
```

After running

```bash
cd ..
```

Current directory becomes

```text
/home/pi
```

---

## Go to the Home Directory

Command:

```bash
cd ~
```

or simply

```bash
cd
```

This takes you directly to your home directory.

---

## Go to the Root Directory

Command:

```bash
cd /
```

The root directory is the highest level of the Linux file system.

---

# File and Directory Operations

---

## Create a New Directory

Command:

```bash
mkdir MyFolder
```

Creates a new directory called **MyFolder**.

---

## Create Multiple Directories

```bash
mkdir Project1 Project2 Project3
```

Creates all three directories simultaneously.

---

## Create Nested Directories

```bash
mkdir -p Project/src/include
```

The `-p` option creates parent directories automatically if they do not already exist.

---

## Create an Empty File

Command:

```bash
touch notes.txt
```

Creates an empty file called `notes.txt`.

If the file already exists, its timestamp is updated.

---

## Copy Files

The `cp` command is used to copy files from one location to another.

General Syntax:

```bash
cp <source_file> <destination>
```

Here,

- `<source_file>` is the file you want to copy.
- `<destination>` is where you want to paste the copied file.

---

### Example 1: Copy a File with a New Name

Command:

```bash
cp notes.txt backup_notes.txt
```

Suppose your current directory contains:

```text
notes.txt
```

After running the above command:

```text
notes.txt
backup_notes.txt
```

A new copy of the file is created in the **same directory**, but with a different name.

---

### Example 2: Copy a File to Another Directory

Command:

```bash
cp notes.txt Documents/
```

Suppose your current directory contains:

```text
notes.txt
Documents/
```

After running the command:

```text
Current Directory
│
├── notes.txt
└── Documents
      └── notes.txt
```

The original file remains in its current location, while a copy is pasted into the **Documents** directory.

---

### Example 3: Copy Multiple Files

```bash
cp file1.txt file2.txt Documents/
```

Both files will be copied into the **Documents** directory.

---

### Example 4: Copy an Entire Directory

Command:

```bash
cp -r Project BackupProject
```

The `-r` option stands for **recursive**.

It copies the entire directory, including all its files and subdirectories.

For example,

Before:

```text
Project/
    main.py
    README.md
```

After running:

```bash
cp -r Project BackupProject
```

You will have:

```text
Project/
BackupProject/
```

Both directories will contain the same files.

---

## Rename a File

Command:

```bash
mv oldname.txt newname.txt
```

Although the `mv` command means **move**, it is also used for renaming files.

---

## Move a File

Command:

```bash
mv notes.txt Documents/
```

Moves the file into the Documents directory.

---

## Delete a File

Command:

```bash
rm notes.txt
```

Deletes the specified file permanently.

> ⚠️ **Warning:** Deleted files cannot be recovered from the Trash using this command.

---

## Delete a Directory

Command:

```bash
rmdir EmptyFolder
```

The directory must be empty.

---

## Delete a Directory with Contents

Command:

```bash
rm -r Project
```

Deletes the directory and everything inside it.

> ⚠️ **Warning:** Be very careful when using `rm -r`.

---

## Force Delete

Command:

```bash
rm -rf Project
```

The `-f` option forces deletion without asking for confirmation.

> ⚠️ **Extreme Caution:** Never run commands such as:
>```bash
> sudo rm -rf /
> ```
> Doing so will destroy your operating system.

---

# Viewing File Contents

---

## Display the Entire File

Command:

```bash
cat filename.txt
```

Example:

```bash
cat notes.txt
```

Displays the entire contents of the file.

---

## Display Large Files Page-by-Page

Command:

```bash
less filename.txt
```

Navigate using:

- Arrow keys
- Page Up
- Page Down

Press **Q** to quit.

---

## Display the First Few Lines

Command:

```bash
head filename.txt
```

Displays the first 10 lines.

Example:

```bash
head launch.py
```

Very useful for checking ROS 2 launch files.

---

## Display the Last Few Lines

Command:

```bash
tail filename.txt
```

Displays the last 10 lines.

---

## Monitor a Growing Log File

Command:

```bash
tail -f log.txt
```

This command continuously updates as new lines are added.

This is particularly useful when:

- Monitoring ROS 2 logs
- Watching sensor outputs
- Debugging applications

Press **Ctrl + C** to stop monitoring.

---

# Searching Files and Text

Searching is one of the most powerful features available in Linux.

As your projects become larger, manually locating files becomes increasingly difficult.

Linux provides several commands that make searching fast and efficient.

---

## Search for Text Inside Files Using grep

Command:

```bash
grep "keyword" filename.txt
```

Example:

```bash
grep "publisher" package.xml
```

Output:

```text
<publisher>Pritam Kalita</publisher>
```

The `grep` command searches for lines containing a specified word or pattern.

It is one of the most frequently used commands by Linux users and ROS 2 developers.

---

## Search Recursively

Command:

```bash
grep -r "laser_scan" .
```

This searches all files inside the current directory and its subdirectories.

Example applications:

- Searching ROS 2 packages
- Finding parameter names
- Searching configuration files
- Locating topic names

---

## Ignore Uppercase and Lowercase Differences

Command:

```bash
grep -i "raspberry" README.md
```

The `-i` option performs a case-insensitive search.

---

## Search for Files by Name

Command:

```bash
find . -name "*.py"
```

Searches for every Python file inside the current directory.

---

## Search the Entire Home Directory

```bash
find ~ -name "package.xml"
```

Very useful for locating ROS 2 packages.

---

## Locate Files Quickly

Command:

```bash
locate package.xml
```

Unlike `find`, the `locate` command searches a pre-built database, making it much faster.

> 💡 **Note:** If `locate` is unavailable, install it using:

```bash
sudo apt install plocate
```

and update its database:

```bash
sudo updatedb
```

---

# File Permissions and Ownership

Linux treats everything as a file, and every file has an owner along with a set of permissions that determine who can read, modify, or execute it.

Understanding file permissions is important because many system configuration files require administrator (root) privileges to modify.

---

## Check File Permissions

Command:

```bash
ls -l
```

Example Output:

```text
-rwxr-xr-- 1 pi pi 3240 Jul 25 hello.py
```

Let's understand what each part means.

| Section | Meaning |
|---------|---------|
| `-` | Regular file |
| `rwx` | Owner permissions |
| `r-x` | Group permissions |
| `r--` | Other users' permissions |

Permission letters:

| Symbol | Meaning |
|---------|---------|
| `r` | Read |
| `w` | Write |
| `x` | Execute |

---

## Change File Permissions

Command:

```bash
chmod permissions filename
```

Example:

```bash
chmod +x hello.py
```

This makes the file executable.

Now you can execute it directly using:

```bash
./hello.py
```

---

## Remove Execute Permission

```bash
chmod -x hello.py
```

---

## Change Permissions Using Numeric Values

Linux also supports numeric permission values.

| Permission | Value |
|------------|------|
| Read | 4 |
| Write | 2 |
| Execute | 1 |

Example:

```bash
chmod 755 hello.py
```

This gives

- Owner → Read, Write, Execute
- Group → Read, Execute
- Others → Read, Execute

---

## Change File Owner

Command:

```bash
sudo chown newuser filename
```

Example:

```bash
sudo chown pi myfile.txt
```

This changes the owner of the file.

---

# Administrator Privileges (`sudo`)

Some Linux commands require administrator (root) privileges.

Instead of logging in as the root user, Linux allows trusted users to temporarily execute commands as the administrator using the `sudo` command.

General Syntax:

```bash
sudo command
```

Example:

```bash
sudo apt update
```

You will usually be prompted to enter your account password before the command is executed.

> 💡 **Note:** Use `sudo` only when necessary. Running commands with administrator privileges can modify important system files.

---

# Installing and Updating Software

Raspberry Pi OS uses the **APT (Advanced Package Tool)** package manager to install, update, and remove software.

---

## Update the Package List

Command:

```bash
sudo apt update
```

This downloads the latest package information from the configured software repositories.

> 💡 **Note:** Always run this command before installing new software.

---

## Upgrade Installed Packages

Command:

```bash
sudo apt upgrade
```

This upgrades all installed software packages to their latest available versions.

---

## Update and Upgrade Together

A very common practice is to execute both commands together.

```bash
sudo apt update && sudo apt upgrade
```

### Understanding the `&&` Symbol

The `&&` operator tells Linux to execute the second command **only if the first command completes successfully**.

Example:

```bash
sudo apt update && sudo apt upgrade
```

Sequence:

1. Update package list.
2. If successful, begin upgrading packages.

If the first command fails, the second command is **not executed**.

This is one of the most commonly used operators in Linux.

---

## Install New Software

General Syntax:

```bash
sudo apt install package_name
```

Example:

```bash
sudo apt install git
```

This installs Git.

Another example:

```bash
sudo apt install htop
```

---

## Install Multiple Packages

```bash
sudo apt install git curl tree
```

All three packages will be installed.

---

## Remove Software

```bash
sudo apt remove package_name
```

Example:

```bash
sudo apt remove htop
```

---

## Remove Unused Packages

```bash
sudo apt autoremove
```

Removes packages that are no longer required.

---

## Search Available Packages

```bash
apt search vscode
```

---

## Display Package Information

```bash
apt show git
```

Displays:

- Version
- Description
- Dependencies
- Package size

---

# Raspberry Pi Configuration

---

## Open Raspberry Pi Configuration Tool

Command:

```bash
sudo raspi-config
```

This opens the Raspberry Pi configuration utility.

Using this tool you can:

- Change the hostname
- Change password
- Configure boot options
- Enable SSH
- Enable VNC
- Enable I2C
- Enable SPI
- Enable UART
- Configure localization
- Configure keyboard layout

This is one of the most useful Raspberry Pi commands.

---

# Raspberry Pi System Information

---

## Display Current Username

```bash
whoami
```

Example Output:

```text
pi
```

---

## Display Current Date and Time

```bash
date
```

---

## Display Current Hostname

```bash
hostname
```

Example Output:

```text
rpi1
```

---

## Display Raspberry Pi IP Address

```bash
hostname -I
```

Example:

```text
192.168.1.145
```

This is probably the fastest way to determine your Raspberry Pi's current IP address.

---

## Display Linux Kernel Version

```bash
uname -a
```

---

## Display Raspberry Pi OS Version

```bash
cat /etc/os-release
```

---

## Display CPU Temperature

```bash
vcgencmd measure_temp
```

Example:

```text
temp=46.2'C
```

Useful for checking whether your Raspberry Pi is overheating.

---

## Check CPU Throttling

```bash
vcgencmd get_throttled
```

This command reports whether the Raspberry Pi has experienced:

- Under-voltage
- CPU throttling
- High temperature

Very useful while debugging hardware issues.

---

# Networking Commands

Networking is one of the most important aspects of Raspberry Pi usage, especially when accessing the board remotely using SSH or TigerVNC.

---

## Display IP Address

Command:

```bash
ip addr
```

This displays all network interfaces.

Look for:

```text
wlan0
```

for Wi-Fi. The number following the word `inet` is your IP address -- just look uptil the slash. 

and

```text
eth0
```

for Ethernet.

---

## Test Network Connectivity

Command:

```bash
ping google.com
```

Example Output:

```text
64 bytes from ...
```

Press

```
Ctrl + C
```

to stop the command.

---

## Ping Another Device

```bash
ping 192.168.1.150
```

Useful for checking whether another device is reachable.

---

## Display Saved Wi-Fi Connections

```bash
nmcli connection show
```

Displays all saved Wi-Fi profiles.

---

## Scan Available Wi-Fi Networks

```bash
nmcli device wifi list
```

Lists all nearby wireless networks.

---

## Connect to a Wi-Fi Network

```bash
sudo nmcli device wifi connect "HomeWiFi" password "mypassword"
```

This command was discussed in detail in **Lesson 5**.

---

## Display Wireless Connection Properties

```bash
iwconfig
```

Displays:

- Wireless mode
- Frequency
- Signal strength
- ESSID

---

## Scan Nearby Wi-Fi Networks

```bash
sudo iwlist wlan0 scan
```

This performs a detailed wireless scan.

---

## Display Only Wi-Fi Names

```bash
sudo iwlist wlan0 scan | grep ESSID
```

### Understanding the `|` (Pipe) Operator

The `|` operator is called a **pipe**.

It sends the output of one command directly into another command.

Example:

```bash
sudo iwlist wlan0 scan | grep ESSID
```

Explanation:

- `iwlist` scans all nearby Wi-Fi networks.
- The output is sent to `grep`.
- `grep` extracts only the lines containing the word **ESSID**.

Without the pipe operator, the output would contain hundreds of lines.

The pipe operator is heavily used in Linux and ROS 2.

Another example:

```bash
history | grep ssh
```

This displays only the previously executed commands containing the word **ssh**.

---

# Command History

Linux automatically stores previously executed commands.

Display command history:

```bash
history
```

Search the history:

```bash
history | grep apt
```
---

# Clear the Terminal

```bash
clear
```

or simply press

```
Ctrl + L
```

Both commands clear the terminal screen without affecting running programs.

---

# Process Management

Whenever you run a program on Linux, it starts as a **process**. Linux provides several commands to monitor and manage running processes.

---

## Display Running Processes

Command:

```bash
ps
```

This displays the processes currently running in your terminal session.

Example Output:

```text
PID TTY          TIME CMD
2531 pts/0    00:00:00 bash
2610 pts/0    00:00:00 ps
```

---

## Display All Running Processes

Command:

```bash
ps -aux
```

This displays all processes currently running on the Raspberry Pi.

Useful information includes:

- Process ID (PID)
- CPU usage
- Memory usage
- Running time
- Command

---

## Monitor Running Processes

Command:

```bash
top
```

The `top` utility continuously updates the display and shows:

- CPU usage
- RAM usage
- Running processes
- Process IDs
- System uptime

Press

```
Q
```

to quit.

---

## Better Process Monitor

Command:

```bash
htop
```

If `htop` is not installed:

```bash
sudo apt install htop
```

Compared to `top`, `htop` provides:

- Colored interface
- Mouse support
- Easier navigation
- Better readability

It is highly recommended.

---

## Stop a Running Process

General Syntax:

```bash
kill PID
```

Example:

```bash
kill 3245
```

Replace `3245` with the actual Process ID.

---

## Force Stop a Process

Sometimes a program refuses to close.

Use:

```bash
kill -9 PID
```

Example:

```bash
kill -9 3245
```

> ⚠️ Use this only when a normal `kill` command does not work.

---

# Monitoring Memory and Disk Usage

---

## Display RAM Usage

Command:

```bash
free -h
```

Example Output:

```text
total   used   free
7.8Gi   1.2Gi  5.9Gi
```

The `-h` option displays values in a human-readable format.

---

## Display Disk Usage

Command:

```bash
df -h
```

Example Output:

```text
Filesystem      Size Used Avail
/dev/root        59G  14G   42G
```

Useful for checking whether your microSD card is running out of storage.

---

## Display Folder Size

General Syntax:

```bash
du -sh folder_name
```

Example:

```bash
du -sh ros2_ws
```

Example Output:

```text
1.8G    ros2_ws
```

Very useful when your ROS 2 workspace starts becoming large.

---

# Redirecting Command Output

Sometimes you may want to save the output of a command to a file instead of displaying it on the terminal.

Linux provides **output redirection** operators for this purpose.

---

## Write Output to a File (`>`)

General Syntax:

```bash
command > filename
```

Example:

```bash
ls > files.txt
```

Instead of displaying the list of files on the terminal, Linux writes it into `files.txt`.

> 💡 **Note:** If the file already exists, its contents will be overwritten.

---

## Append Output to a File (`>>`)

General Syntax:

```bash
command >> filename
```

Example:

```bash
date >> logfile.txt
```

Each time the command is executed, the new output is added to the end of the file.

This is commonly used while maintaining logs.

---

## View the Saved Output

```bash
cat logfile.txt
```

---

# The `tee` Command

Sometimes you may want to:

- View the output on the terminal
- Save it to a file at the same time

This can be achieved using the `tee` command.

Example:

```bash
ls | tee files.txt
```

The output appears on the terminal and is simultaneously written to `files.txt`.

---

# Wildcards

Wildcards help you work with multiple files simultaneously.

---

## The `*` Wildcard

Matches zero or more characters.

Example:

```bash
ls *.py
```

Displays all Python files.

Another example:

```bash
rm *.log
```

Deletes all log files.

---

## The `?` Wildcard

Matches exactly one character.

Example:

```bash
ls file?.txt
```

Matches:

```text
file1.txt
file2.txt
fileA.txt
```

but not

```text
file10.txt
```

---

# Watching a Command Continuously

Command:

```bash
watch command
```

Example:

```bash
watch free -h
```

This updates the memory usage every two seconds.

Another useful example:

```bash
watch vcgencmd measure_temp
```

This continuously displays the Raspberry Pi CPU temperature.

Press

```
Ctrl + C
```

to stop.

---

# Services in Raspberry Pi

Linux runs many programs as background services.

---

## Check Service Status

General Syntax:

```bash
systemctl status service_name
```

Example:

```bash
systemctl status ssh
```

---

## Start a Service

```bash
sudo systemctl start ssh
```

---

## Stop a Service

```bash
sudo systemctl stop ssh
```

---

## Restart a Service

```bash
sudo systemctl restart ssh
```

---

## Enable a Service During Boot

```bash
sudo systemctl enable ssh
```

---

## Disable a Service

```bash
sudo systemctl disable ssh
```

---

# Safe Shutdown and Reboot

One of the most important habits when using a Raspberry Pi is **always shutting it down properly**.

Unlike many desktop computers, abruptly disconnecting power from the Raspberry Pi can corrupt the operating system stored on the microSD card.

---

## Shut Down Immediately

Command:

```bash
sudo shutdown now
```

The Raspberry Pi safely closes all running programs before shutting down.

Wait until the green activity LED stops blinking before disconnecting the power supply.

---

## Power Off

Command:

```bash
sudo poweroff
```

This command performs the same task as `shutdown now`.

---

## Restart the Raspberry Pi

Command:

```bash
sudo reboot
```

The operating system safely closes all running programs and restarts.

This is one of the most frequently used Raspberry Pi commands.

---

## Schedule a Shutdown

Command:

```bash
sudo shutdown +10
```

The Raspberry Pi will shut down after 10 minutes.

---

## Cancel a Scheduled Shutdown

Command:

```bash
sudo shutdown -c
```

---

# Useful Keyboard Shortcuts

| Shortcut | Purpose |
|-----------|----------|
| Ctrl + C | Stop current program |
| Ctrl + L | Clear terminal |
| Ctrl + D | Logout / Exit terminal |
| Ctrl + R | Search command history |
| Up Arrow | Previous command |
| Down Arrow | Next command |
| Tab | Auto-complete commands and filenames |

Learning these shortcuts can significantly improve your productivity.

---

# Daily Raspberry Pi Command Cheat Sheet

| Task | Command |
|------|---------|
| Current directory | `pwd` |
| List files | `ls` |
| Show hidden files | `ls -la` |
| Copy files | `cp` |
| Move/Rename | `mv` |
| Delete file | `rm` |
| Delete directory | `rm -r` |
| Search text | `grep` |
| Search files | `find` |
| Install software | `sudo apt install` |
| Update package list | `sudo apt update` |
| Upgrade packages | `sudo apt upgrade` |
| Raspberry Pi settings | `sudo raspi-config` |
| IP Address | `hostname -I` |
| Wi-Fi Scan | `nmcli device wifi list` |
| View saved Wi-Fi | `nmcli connection show` |
| RAM Usage | `free -h` |
| Disk Usage | `df -h` |
| Folder Size | `du -sh` |
| CPU Temperature | `vcgencmd measure_temp` |
| Running Processes | `top` |
| Better Process Monitor | `htop` |
| Shutdown | `sudo shutdown now` |
| Reboot | `sudo reboot` |

---

Congratulations! 🎉

You are now familiar with many of the Linux terminal commands that Raspberry Pi users rely on in their day-to-day work. As you continue exploring Raspberry Pi, these commands will become an essential part of your daily workflow and significantly improve your efficiency while developing, debugging, and managing your projects.

---

**Happy Learning!** 😊