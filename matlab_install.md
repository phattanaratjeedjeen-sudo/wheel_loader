## Matlab Install

This instruction used for installing matlab R2025b on ubuntu 24.04

1. Unzipping the Installer
Navigate to your folder and extract the downloaded ZIP file:

    ```bash
    unzip -x -a -K [file] -d matlab_R2025b_installer 
    ```

2. Running the Installer
To launch the graphical installer, you need root privileges and permission to display the GUI:

    ```bash
    # Enter root mode
    sudo su 
    ```

    ```bash
    # Grant root GUI access
    xhost +si:localuser:root 
    ```

    ```bash
    # Run the installer script
    cd matlab_R2025b_installer
    sudo ./install 
    exit
    ```

3. Activating License
    In recent MATLAB versions, the standalone activation script may not be present. Instead, you can activate the software by launching MATLAB directly. If a license is not found, the activation GUI will appear automatically.

    ```bash
    # Navigate to your install bin
    cd /usr/local/MATLAB/R2025b/bin/
    ```

    ```bash
    # Run MATLAB to trigger the activation prompt
    ./matlab 
    ```

4. Launching MATLAB
    To launch MATLAB from anywhere without typing the full path, create a symbolic link to your system's binary directory:
    
    ```bash
    sudo ln -s /usr/local/MATLAB/R2025b/bin/matlab /usr/local/bin/matlab
    ```

    Once linked, you can launch MATLAB directly from the terminal:
    
    ```bash
    matlab 
    ```
