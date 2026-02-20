# **ENGR 290 - Technical Assignment 1**  
### **Team 1**  

## **Team Members**  
- **Alexandre Nasseri** - 40281158  
- **Callum Gagnon**  - 40255747
- **Frederic Gagne** - 40278058  
- **Jose Del Portillo Neira** - 40295303 
- **Maxime Genion-Perron**  -40246477
- **Zaid Amer**  

---

## **Project Overview**  
This project integrates two sensors—an **ultrasonic sensor** and an **infrared sensor**—to measure the distance of an object. Based on the measured distance, the brightness of an **LED** is dynamically adjusted:  

- **≤ 12 cm** → LED brightness set to **100%**  
- **≥ 46 cm** → LED brightness set to **10%**  
- **12 cm to 46 cm** → LED brightness decreases **linearly**  

If the measured distance is **out of range**, a secondary **LED blinks** with a period of **T = 0.5s** to indicate an error.  

The user can switch between the **ultrasonic** and **infrared** sensors, ensuring adaptability in different environments.  

---

## **Files & Descriptions**  

- **`TechnicalAssignment1_IR_290.ino`**  
  - Code for the **infrared sensor** operation.  

- **`TechnicalAssignment1_US_290.ino`**  
  - Code for the **ultrasonic sensor** operation.  

---

