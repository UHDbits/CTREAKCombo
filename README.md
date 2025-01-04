# Setting Up Code Environment
> [!NOTE]
> This repository is made to be more usable in IntelliJ, but it can be used in Visual Studio Code. In VS Code, you won't be able to run code replay or run performance profiling without going through the terminal.
<details>
<summary>IntelliJ Setup</summary>

1. Install [IntelliJ IDEA Community Edition (free) or IntelliJ IDEA Ultimate (paid, free through GitHub Student Developer Pack)](https://www.jetbrains.com/idea/download). Make sure you have the latest version of WPILib 2025 installed as well.
2. Add the [FRC IntelliJ plugin](https://plugins.jetbrains.com/plugin/9405-frc) and the [google-java-format plugin](https://plugins.jetbrains.com/plugin/8527-google-java-format). Both of these can be installed in the IDE directly by going through File -> Settings -> Plugins.
3. Clone this repository using your favorite Git client, and open it in IntelliJ.
4. Go to File -> Project Structure, and hit the "Edit" button next to SDK. Press the "plus" icon, press "Add JDK from disk...", and go to the directory "C:\Users\Public\wpilib\2025\jdk" and add it. This ensures that we are using the officially supported JDK version from WPILib, to hopefully minimize issues.
5. You should see a prompt in the bottom right corner of IntelliJ asking you to enable google-java-format, if so, click "Enable for this project". If not, go to "File -> Settings -> google-java-format settings", and enable it.
6. Run the "Build Robot" run configuration once, just to make sure that you've downloaded all dependencies.
</details>
