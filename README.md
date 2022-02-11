![Augmented Hip](https://github.com/hyblocker/augmented-hip/blob/master/github/banner.png?raw=true)

>  Augmented Hip, a new hip tracker, which estimates your hip's position and orientation from your feet and head.

### Demonstration (Vive Trackers)
[![Watch the video](https://raytracing-benchmarks.are-really.cool/8HCBotk.png)](https://raytracing-benchmarks.are-really.cool/2kKtVWe.mp4)

### Demonstration (Kinect)
[![Watch the video](https://raytracing-benchmarks.are-really.cool/3MV8wyj.png)](https://raytracing-benchmarks.are-really.cool/7py8Dny.mp4)

# Install

### Method 1

- Download the zip file from [Releases](https://github.com/hyblocker/augmented-hip/releases/tag/0.1)
- Extract it to any folder
- Press  the **Windows Key** + **R** to open the Run dialog, and paste in `%localappdata%/openvr` then press **Enter** to open the folder in a file explorer.
- Open the file `openvrpaths.vrpath` with notepad.
- Add the **FULL FILE PATH** to the `"external_drivers"` block, as show below:
- > ⚠ Make sure that the file **ONLY CONTAINS DOUBLE BACKSLASHES** `\\`
  >
  > You can also use **forward slashes** `/`
```json
{
	"config" : 
	[
		"C:\\Program Files (x86)\\Steam\\config"
	],
	"external_drivers" : 
	[
		"C:\\VR\\OWOTRACK\\driver",
		"C:\\K2EX\\KinectToVR",
		"C:\\VR\\AugmentedHip\\driver"
	],
	"jsonid" : "vrpathreg",
	"log" : 
	[
		"C:\\Program Files (x86)\\Steam\\logs"
	],
	"runtime" : 
	[
		"C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR"
	],
	"version" : 1
}
```

### Method 2

You can also install it into the direct drivers folder.

Access the SteamVR folder

![opening steamvr local files](https://raytracing-benchmarks.are-really.cool/Af6eSnS.png)

Go to the `drivers` folder and create a new folder with whatever name you want. Maybe `augmented-hip`

Copy the files and folders from the zip file in [Releases](https://github.com/hyblocker/augmented-hip/releases/tag/0.1):
```
bin
resources
driver.vrdrivermanifest
```
into that newly created folder.

>  *An installer will be coming soon*

# Troubleshooting

## My hip flips when I slightly turn around!

Ensure that the tracker on your left foot is using the "Left Foot" tracker role, and, also, that the tracker on your right foot is using using the "Right Foot" tracker role.

If you can find the menu to do so by opening SteamVR settings and going to the **Manage Vive Trackers** section under the **Controllers** tab.

You can figure out which tracker is which by clicking the "Identify" button.

![Opening the tracker roles page](https://raytracing-benchmarks.are-really.cool/3Vk9xt4.png)

## AugmentedHip doesn't appear in my SteamVR add-ons list

Make sure that your `openvrpaths.vrpath` file doesn't contain malformed JSON. If you still have issues join the [KinectToVR Discord server](https://discord.gg/YBQCRDG) for support in the `#aughip-help` channel.

## I have another problem!

Join the [KinectToVR Discord server](https://discord.gg/YBQCRDG) for support in the `#aughip-help` channel.

## License

This SteamVR Add-on  is licensed under GPL-3