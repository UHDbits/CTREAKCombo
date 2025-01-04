# Setting Up Code Environment

> [!NOTE]
> This code repository is meant to be used in IntelliJ, but it can be used in Visual Studio Code. In VS Code, you won't be able to run code replay or run performance profiling without going through the terminal.

<details>
<summary>IntelliJ Setup</summary>

1. Install [IntelliJ IDEA Community Edition (free) or IntelliJ IDEA Ultimate (paid, free through GitHub Student Developer Pack)](https://www.jetbrains.com/idea/download). Make sure you have the latest version of WPILib 2025 installed as well.
2. Add the [FRC IntelliJ plugin](https://plugins.jetbrains.com/plugin/9405-frc). This can be installed in the IDE directly by going through File -> Settings -> Plugins.
3. Clone this repository using your favorite Git client, and open it in IntelliJ.
4. Go to File -> Project Structure, and hit the "Edit" button next to SDK. Press the "plus" icon, press "Add JDK from disk...", and go to the directory "C:\Users\Public\wpilib\2025\jdk" and add it. This ensures that we are using the officially supported JDK version from WPILib, to hopefully minimize issues.
5. Run the "Build Robot" run configuration once, just to make sure that you've downloaded all dependencies.
6. (Optional) Add the [google-java-format plugin](https://plugins.jetbrains.com/plugin/8527-google-java-format) for proper code formatting, or, just let spotless handle it when building.
7. (Optional) You should see a prompt in the bottom right corner of IntelliJ asking you to enable google-java-format, if so, click "Enable for this project". If not, go to "File -> Settings -> google-java-format settings", and enable it.
</details>
<details>
<summary>VS Code Setup</summary>

8. Install the latest version of [WPILib](https://github.com/wpilibsuite/allwpilib/releases) and it's built-in version of VS Code.
9. Clone this repository using your favorite Git client, and open it in VS Code.
10. Build the project once to make sure that all dependencies have been downloaded.
11. (Optional) Add the [google-java-format extension](https://marketplace.visualstudio.com/items?itemName=JoseVSeb.google-java-format-for-vs-code) through VS Code's built in extension manager.
12. (Optional) When/if you format code for the first time, make sure you select "Google Java Format for VS Code". Or, don't format, and just let spotless handle it when building.
</details>

# Run Configurations
## Build Robot