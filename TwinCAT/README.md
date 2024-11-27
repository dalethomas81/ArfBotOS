# TwinCAT
I thought it would be fun to convert ArfBotOS over to TwinCAT since TwinCAT is what I primarily use in my career. So, here is the folder that will be dedicated to that.  

## Architecture Overview
For testing, we will develop on the same PC that the TwinCAT PLC and HMI runs. The NIC on the PC will be used as the EtherCAT master and connect to the robot using a EtherCAT slave by Microchip (EVB-LAN9252-SPI).  

## PLC Installation
_Note: you will need a free Beckhoff account to download the software required. Additionally, you will need Visual Studio 2022 installed with the C++ option selected during installation._
1. Download and install the [TwinCAT Package Manager](https://www.beckhoff.com/en-us/support/download-finder/search-result/?download_group=725136885&download_item=725320261)
2. Open the TwinCAT Package Manager and click on the Import system config button and select the configuration located here: `\ArfBotOS-TwinCAT\TwinCAT\PackageManager.config`. This will automatically install of the Beckhoff software needed to run ArfBotOS. _Note: this step will take quite a while._
3. Open a command prompt as administrator and run the following command: `bcdedit /set testsigning yes`. This will make it possible to run the C++ module without a signed driver.
4. Run this command and restart Windows: `"C:\Program Files (x86)\Beckhoff\TwinCAT\3.1\System\win8settick.bat"`. This will allow TwinCAT to use the cores of the PC as realtime cores. 
5. After Windows restarts, you should be able to open the solution file in Visual Studio located here: `\ArfBotOS-TwinCAT\TwinCAT\ArfBotOS-TwinCAT\ArfBotOS-TwinCAT.sln`.
6. On the top bar select Extensions->PLC->License Repository and click Install to install the PackML library located here: `C:\Users\dale\Documents\GitHub\ArfBotOS-TwinCAT\TwinCAT\Libraries\OMAC_PackML_State_Machine.library`. _Note: this library will not be needed in the future once the code is converted to use the TwinCAT PackML library._
7. From the top bar select Extensions->TwinCAT-Software Protection and create a new OEM signing certificate with the name `ArfBotOS` making sure to select the option to Sign TwinCAT C++ executables. Use the password `1` when prompted. This will create a custom certificate to use for testing.
8. Open the realtime driver installer here: `C:\Program Files (x86)\Beckhoff\TwinCAT\3.1\System]TcRteInstall.exe` and install the driver for your local ethernet NIC. _Note: it may say incompatible for your NIC. This will most likely still work for testing but drop out every once in a while. A PC with a compatible Intel NIC will be required for production._
9. In the solution explorer, double-click on Device 1 to get to properties. Here you will click on the NIC that you just installed the realtime driver for. _Note: this will be specific to each PC and will have to be reconfigured for each installation._
10. Try building the solution from the build menu in the top bar.
11. After the solution build successfully, activate configuration on your local runtime by selecting Extensions->TwinCAT-Activate Configuration from the top bar. Make sure the target is set to local. _Note: you may be asked ot create trial licenses at this point._Note
12. If activating succeeded, you can go online with the runtime. _Note: depending on if you checked the option to "Autostart PLC Boot Project(s)" or not, you may need to download the PLC program in the next step._
13. From the top bar choose Extensions->PLC->Login. _Note: if prompted about "Port_851" not existing, confirm proceeding with downloading.
14. Now start the runtime by choosing Extensions->PLC->Start from the top bar.
15. At this point, you should be online with the PLC and the runtime should be in run mode.

_Hint: right-click the top bar and add the following toolbars to make life easier: TwinCAT XAE Project Variants and TwinCAT PLC._

## HMI Installation
TODO

### References
https://infosys.beckhoff.com/content/1033/tc3_c/110693003.html?id=5665731665519473284

## Issues
Currently, the TF5113 Advanced Motion pack is needed to perform the kinematics. However, due to the war the EU has restricted and company to share software that can be used to create a drone without a purchase order (normally, Beckhoff shares all software for free).  

To get around this issue, we are going to create a custom kinematics resolver in C++ and load that as a driver to use in the PLC runtime.  

I found a fantastic library created by someone for the Arduino that uses "joint screw axes" (screw theory) as an alternative to "D-H parameters". I was able to port the library over into something that can be used in TwinCAT (keeping the license file in the header so that the original auther can get credit!)  

### References
https://infosys.beckhoff.com/content/1033/tf5400_tc3_advanced_motion_pack/8403043979.html?id=9115801577672301435
https://www.reddit.com/r/TwinCat/s/Dxyvlc9F6e
https://github.com/kousheekc/Kinematics