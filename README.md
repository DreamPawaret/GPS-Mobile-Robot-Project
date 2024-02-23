# GPS-Mobile-Robot-Project

อธิบายไฟล์ต่างๆที่ใช้ในโครงงาน
main : โปรแกรมหลักในการทำงาน ใช้กับบอร์ด Arduino
blynk_interface : โปรแกรมการสำหรับทำงานของบอร์ด node mcu

โค้ดข้างล่างนี้เป็นการเขียนแยกการทำงานแต่ละส่วน ซึ่งเอามาใช้งานจริงใน main
BTS_motor_driver : ใช้ทดสอบการทำงานของ motor driver

compass : ใช้ทดสอบการทำงานของเข็มทิศ

GPS : ทดสอบการทำงานของ gps โดยเอาเฉพาะค่า latitude longitude

gpsRawData : ทดสอบการทำงานของ gps เป็นค่าทั้งหมดที่ได้รับจาก gps ต้องเอาไปฟิลเตอร์ค่าที่เราอยากได้อีกที

rotate_angle : ทดสอบการคำนวณมุมที่หุ่นต้องหันไปยังพิกัดเป้าหมาย

ultrasonic_interrupt : ทดสอบการทำงานของ ultra sonic โดยใช้การ interrupt

โค้ดที่เหลือนอกเหนือจากที่กล่าวมาเป็นการทดลองเขียน 

แหล่งอ้างอิงที่ใช้ในการทำโครงงาน
Arduino Programming the HC-SR04 Project Interrupt Driven Software
https://homediyelectronics.com/projects/arduino/arduinoprogramminghcsr04withinterrupts/?p=4

Datasheet
https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf

Arduino Mega 2560 Rev3 Pinout, Atmega2560 Pin Mapping, EAGLE Files, Schematics, and More! - Documents - Arduino - element14 Community
https://community.element14.com/products/arduino/w/documents/2969/arduino-mega-2560-rev3-pinout-atmega2560-pin-mapping-eagle-files-schematics-and-more

HC-SR04
https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf

Internet of Things (IoT): BLYNK Framework for Smart Home | KnE Social Sciences
https://knepublishing.com/index.php/Kne-Social/article/view/4128/8495#:~:text=Blynk%20is%20an%20IoT%20platform,address%20on%20the%20available%20widgets.

NEO-6M GPS Howto: boost receiving speed (NEO-6, NEO-7, NEO-8) | by Victor | Medium
https://cglabs.medium.com/neo-6m-gps-howto-boost-data-receiving-speed-neo-6-neo-7-neo-8-487275eff1c6

How to Build a GPS Guided Robot : 9 Steps (with Pictures) - Instructables
https://www.instructables.com/How-to-Build-a-GPS-Guided-Robot/

GPS Guides Robotic Car - Circuit Cellar
https://circuitcellar.com/research-design-hub/gps-guides-robotic-car-2/

Draw a circle with a radius on a map
https://www.mapdevelopers.com/draw-circle-tool.php

arduino - WiFi Status 1 on ESP8266 - Stack Overflow
https://stackoverflow.com/questions/69543053/wifi-status-1-on-esp8266

วิธีส่งข้อมูลระหว่างบอร์ด Arduino และ ESP8266 หรือ nodemcu - ModuleMore ขาย arduino และอุปกรณ์/เซนเซอร์/โมดูล สินค้ารับ�
https://www.modulemore.com/article/23/%E0%B8%A7%E0%B8%B4%E0%B8%98%E0%B8%B5%E0%B8%AA%E0%B9%88%E0%B8%87%E0%B8%82%E0%B9%89%E0%B8%AD%E0%B8%A1%E0%B8%B9%E0%B8%A5%E0%B8%AB%E0%B8%B2%E0%B8%81%E0%B8%B1%E0%B8%99%E0%B8%A3%E0%B8%B0%E0%B8%AB%E0%B8%A7%E0%B9%88%E0%B8%B2%E0%B8%87%E0%B8%9A%E0%B8%AD%E0%B8%A3%E0%B9%8C%E0%B8%94-arduino-%E0%B9%81%E0%B8%A5%E0%B8%B0-esp8266-%E0%B8%AB%E0%B8%A3%E0%B8%B7%E0%B8%AD-nodemcu

project_IdDoc322_IdPro739.pdf
https://eng.kps.ku.ac.th/dblibv2/fileupload/project_IdDoc322_IdPro739.pdf

Track ME - Arduino Project Hub
https://create.arduino.cc/projecthub/Tiobel/track-me-457576?ref=tag&ref_id=gps&offset=4

Geographic Distance and Azimuth Calculations | CodeGuru
https://www.codeguru.com/cplusplus/geographic-distance-and-azimuth-calculations/

Latitude Longitude Distance Calculator
https://www.omnicalculator.com/other/latitude-longitude-distance

Azimuth Calculator
https://www.omnicalculator.com/other/azimuth

How to Interface Arduino Mega with NEO-6M GPS Module - Arduino Project Hub
https://create.arduino.cc/projecthub/ruchir1674/how-to-interface-arduino-mega-with-neo-6m-gps-module-1b7283

