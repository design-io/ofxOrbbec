# ofxOrbbec
An openFrameworks addon for Orbbec depth cameras

### Designed for OF 0.12.0 

### Linux Usage 
**Copy Shared Objects**<br />
OF currently doesn't copy .so library files into the `YOUR_PROJECT/bin/libs/` folder automatically. <br />So you will have to manualy make a libs folder at the same level as your executable and add the contents of `libs/orbbec/lib/linux64/` to that folder. 
<br /><br />
**Add ofxOrbbec and ofxOpenCV**<br />
Add the above two addons to your `addons.make` file and your should be good to go. 
<br /><br />
**Copy src folder from example**<br />
Copy the src folder from the example/ folder to test the api  

### Mac Usage 
**Use the project generator**<br />
Use the project generator to add ofxOrbbec to your project or import the example project. <br />
NOTE: Mac cannot use as many of the cameras as Linux or Windows. See the supported cameras in the list here: https://www.orbbec.com/developers/orbbec-sdk/  

### Windows Usage 
**Use the project generator**<br />
Use the project generator to add ofxOrbbec to your project or import the example project. <br />
If the OrbbecSDK.dll and the depthengine_2_0.dll is not in the bin/ folder of your project copy it over from libs/orbbec/lib/vs/x64/ 

### Example output 

![example](https://github.com/design-io/ofxOrbbec/assets/144000/0658f270-c2ff-4dee-bf5b-fa394e4b3ad5)
