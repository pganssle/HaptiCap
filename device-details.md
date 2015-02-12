---
layout: default
title: HaptiCapMag&#58; Device Details
pageline: Device Details
---
<div class="img-container" style="float:left; width:260px">
<a href="images/photos/high-res/HaptiCap 019 - Inside View Annotated.png"><img src="images/photos/low-res/HaptiCap LowRes 019 - Inside View Annotated.png" width="250px" hspace="5px"/></a>
<span class="caption" style="width:260px"><b>Fig. 1</b> An annotated view of the inside of the hat containing the
magnetometer (<b>A</b>), battery holder (<b>B</b>), step-up transformer(<b>C</b>), and Arduino (<b>D</b>)</span>
</div>

The hat itself contains 8 motors, which are circled and labeled in <b>Fig. 1</b>. I've numbered the motors counter-clockwise starting at the motor directly in front of the hat; this numbering relates to how the software selects which motor is running -- the pinouts can be assigned arbitrarily in software.

In addition to the motors, there are three basic components: the sensor (an [HMC5883L](https://github.com/pganssle/HMC5883L) 3-axis digital magnetometer - <b>Fig. 1A</b>), the power source (a cell-phone battery and DC-DC step-up transformer, <b>Fig. 1B</b> & <b>1C</b>), and the microcontroller (Arduino Nano, <b>Fig 1D</b>). In this page, I'll go into more details about each of the components and the reasons for the design choices I made.

### Power source
The power in my prototype is supplied by an 1800 mAh battery from a Sprint HTC Evo 4G<sup><a name="fnote1" ref="#ref1">1</a></sup>, mounted in a wall-charging unit that I disassembled and repurposed. Since Li+ batteries like this one are generally 3.7 V, the voltage is stepped up with a DC-DC step-up transformer, which costs around $5 and generally represents only about 5% power loss.

### Vibration Motors

#### Casing
I bought the motors I'm using some time ago and I don't remember the exact model number. I would guess that nearly any small DC vibration motors should work, so long as you can supply the appropriate current and voltage. One problem with most bare vibration motors is that the unbalanced weight is exposed, and the rotation can easily be stopped by mild pressure.

<div class="img-container" align="center">
<a href="images/photos/high-res/HaptiCap 010 - Vibrator Motor Bare.png"><img src="images/photos/low-res/HaptiCap LowRes 010 - Vibrator Motor Bare.png" alt="Bare vibrator motor" width="500px" align="center"/></a><br>
<span class="caption"><b>Fig. 2</b> A bare vibration motor</span>
</div>

In order to use it in the HaptiCap, it needs to be in a rigid shell. If you have a 3D printer available, you can make a custom shell, but I've found that -- at least with the motors I have -- the diameter of the motors almost exactly matches the inner diameter of a pencil tube, and so I was able to make 8 shells but cutting up a disused mechanical pencil.<sup><a name="fnote2" href="#ref2">1</a></sup>.

<div class="img-container" align="center">
<a href="images/photos/high-res/HaptiCap 012 - North Motor Pocket.png"><img src="images/photos/low-res/HaptiCap LowRes 012 - North Motor Pocket.png" alt="Cased vibrator motor" width="400px" align="center"/></a><br>
<span class="caption"><b>Fig. 3</b> The vibrator motor in a case.</span>
</div>

Another alternative is to use a [button type vibration motor](http://www.amazon.com/Diameter-Button-3V-4-5V-CellPhone-Vibrator/dp/B00H4PM1WO), which already has a rigid casing. I have some of these but I have not tested them yet, so I am not sure if the vibration is strong enough to be of use in this application.

#### Current draw
In the motors I'm using, I've measured a resistive load of ~25 Ω, and assuming they are designed to work with 3 V, that's a constant current draw of 120 mA with a 100% duty cycle. This is considerably more than the 40 mA specified in the <a href="http://arduino.cc/en/Main/arduinoBoardNano">Arduino Nano spec</a>, so it may be best to try a design using a [multiplexer](https://www.sparkfun.com/products/9056) rather than using the output pins directly<sup><a name="fnote3" href="#ref3">2</a></sup>. 

<div align="right" class="img-container" style="float:right; width:250px"><img src="images/photos/high-res/HaptiCap 017 - Thermal%20Image%20Arduino.jpg" /><br>
<span class="caption"><b>Fig. 4</b> Thermal image of the Arduino Nano after several hours of continous use.</span>
</div>

I'm actually using a [SainSmart Nano 3.0](http://www.amazon.com/SainSmart-Nano-v3-0-Compatible-Arduino/dp/B00761NDHI/) (Arduino clone), and empirically it seems to be capable of providing around 100 mA of current, and in practice I tend to run the motors with close to a 50% duty cycle, so the continuous draw is generally ~50 mA. Using a thermal camera, I noticed that the processor runs a bit hot (~60 °C), but this is not uncomfortable, and doesn't seem to be causing any damage to the processor itself.

Interestingly, either the microcontroller is extremely inefficient at supplying power to the motors or the motors are *not* the largest source of current draw in the system (I strongly suspect the former). Even running with a 50% duty cycle (updated every 20 ms), a freshly charged 1800 mAH battery only lasts about 5.5 hours, indicating that the current draw is close to 325 mA, while the motors are only drawing an average of ~ 50 mA. I plan to investigate this with a current probe and some experiments with the algorithms to determine what the main cause of current draw is to try and get the current draw closer to 100 mA.

#### Footnotes
<ol class="footnotes">
<li><a href="#fnote1" name="ref1">^</a> Originally I was using the batteries from my old phone, but these were getting a bit long in the tooth and some of them had started to bulge out a bit and fray, so I didn't want to keep using them in such close proximity to my head. I ended up buying 3 more along with a wall charger from eBay for about $15.
<li><a href="#fnote2" name="ref2">^</a> I couldn't find my <a href="https://en.wikipedia.org/wiki/Rotary_tool">Dremel</a> and <a href="https://en.wikipedia.org/wiki/Grinding_wheel#Cut_off_wheels">cut-off wheels</a>, so I heated up a <a href="https://en.wikipedia.org/wiki/Utility_knife">box-cutter</a> with a lighter to rapidly cut the plastic.
<li><a href="#fnote3" name="ref3">^</a> This is also useful if you have a microcontroller with fewer outputs than you have motors - the Nano has 8-14 digital outputs (depending on how you want to count). A digital multiplexer can control 8 motors with 3 digital I/O pins, and 16 motors with 4 digital I/O pins.
</ol>

