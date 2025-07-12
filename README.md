# CPE C401 - Computer Architecture Projects  
> **Jose Rizal University**  
> Submitted for: CPE C401–301G  
> Instructor: Engr. Amor S. Ellares  
>  
> By: Rizelle Bautista  
> Date: July 2025

---

## 📁 Table of Contents
- [📌 Introduction](#-introduction)
- [🛠️ Project Descriptions](#️-project-descriptions)
  - [🔢 Midterm: 4-bit Adder/Subtractor Circuit](#-midterm-4-bit-addersubtractor-circuit)
  - [🧠 Finals: 16-bit CPU Visualizer using Arduino](#-finals-16-bit-cpu-visualizer-using-arduino)
- [📦 Materials Used](#-materials-used)
- [📽️ Video Demonstrations](#️-video-demonstrations)
- [💬 Reflections](#-reflections)
- [🔗 Source Files](#-source-files)

---

## 📌 Introduction

This repository showcases the **Midterm and Final Projects** completed for the course **Computer Architecture (CPE C401)**. Both projects reflect our understanding of digital systems, instruction cycle simulation, and CPU design principles, implemented using hardware such as logic ICs and Arduino boards.

---

## 🛠️ Project Descriptions

### 🔢 Midterm: 4-bit Adder/Subtractor Circuit

A breadboard-based logic project that performs addition and subtraction of 4-bit binary numbers.

**Features:**
- Addition and Subtraction modes using a toggle switch
- 7-segment display output in decimal
- LED binary output
- Handles two’s complement for negative results

**Technologies Used:**
- Logic Gates (XOR, Adders)
- 7-Segment Display (via CD4511 decoder)
- Manual binary input via DIP switches

📄 [View Midterm Report](./P1%20-%20MIDTERM%20PROJECT_GROUP%206.pdf)

---

### 🧠 Finals: 16-bit CPU Visualizer using Arduino

An interactive system built on the Arduino Mega to simulate a CPU instruction cycle and execution visualization.

**Features:**
- Manual & automatic instruction execution
- 4-register CPU with 16-bit operations
- LCD status monitor for PC, instruction, register values
- LED array for Program Counter visual
- 7-Segment for displaying values

**Demonstrated Programs:**
- Basic Math (e.g., (25+13)\*2 - 10)
- Two’s Complement
- Large 16-bit operations
- Bitwise logic (AND, OR, XOR)

📄 [View Final Report](./FINAL%20PROJECT_GROUP%206.pdf)

---

## 📦 Materials Used

| Component                      | Midterm | Final |
|-------------------------------|---------|-------|
| Breadboard                    | ✅      | ✅    |
| 74LS83 Adder IC               | ✅      | ❌    |
| CD4511 Decoder                | ✅      | ❌    |
| LEDs                          | ✅      | ✅ (16) |
| 7-Segment Display             | ✅ (1-digit) | ✅ (4-digit) |
| SPDT Switch                   | ✅      | ❌    |
| Arduino Mega 2560             | ❌      | ✅    |
| LCD I2C 20x4                  | ❌      | ✅    |
| Dual Axis Joystick            | ❌      | ✅    |
| Push Buttons                  | ❌      | ✅ (4) |

---

## 📽️ Video Demonstrations

📹 **Midterm Demo:**  
🔗 [Google Drive Link](https://drive.google.com/drive/folders/1RMc4Qei7JrQ7bIN52P4YAGsYqWnEMIDE?usp=sharing)

📹 **Finals Demo:**  
🔗 [Google Drive Link](https://drive.google.com/drive/folders/1T4-UrmAEzlvAfNgcdg3u1AnegezzepB1?usp=sharing)

---

## 💬 Reflections

### 🧩 Midterm Reflection
> We learned that subtraction in binary must be handled via two’s complement representation. While it was challenging to display negative outputs on a single-digit 7-segment display, we gained deeper understanding of binary arithmetic and its physical implementation. Extending this system to 2-digit display would require more complex logic or microcontroller support.

### 🧠 Finals Reflection
> This project gave us a deeper appreciation of how CPUs function. By visualizing each instruction step and its effect on registers and memory, we better understood the fetch-decode-execute cycle. Building the interface also improved our hardware/software integration skills, from LED outputs to interactive controls via joystick and buttons.

---

## 🔗 Source Files

📁 [Final Project Arduino Source Code](https://drive.google.com/drive/folders/1orB-cYulPq6-g1cP0JWeY5NSkJdy6ehK?usp=sharing)
