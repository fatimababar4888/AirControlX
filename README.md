# AirControlX ✈️ – Airport Traffic Simulation & Violation Management System

**By:** Aliza Rashid & Fatima Babar  
**Semester Project – CS-E (Module 3)**

---

## 🧭 Overview

AirControlX is a multithreaded, multiprocess simulation of air traffic control designed to model and manage flight operations at an airport. The simulation supports real-time graphics rendering using SFML, handles inter-process communication (IPC), and implements violation detection with automatic notice (AVN) generation and a payment portal.

---

## 🚀 Features

- 🛬 **Dynamic Flight Scheduling**  
  Flights of different types (Commercial, Cargo, Emergency) with random fuel levels, directions, and priorities.

- 🛫 **Runway Assignment & Traffic Management**  
  Intelligent runway allocation based on flight direction and emergency status.

- 💡 **Violation Detection System**  
  - Speed violations  
  - Cargo misuse  
  - Ground faults

- ⚠️ **AVN (Airspace Violation Notices)**  
  Automatic AVN issuance with due dates and fine calculation including service charges.

- 🧾 **StripePay-Inspired Payment System**  
  Simulated third-party payment processing with 90% success probability.

- 📊 **Live Dashboard & SFML GUI**  
  Real-time updates of active flights, AVNs, queues, and more.

- 💬 **IPC-based Subsystems**  
  Forked child processes for:
  - ATCS Controller  
  - AVN Generator  
  - Airline Portal  
  - StripePay

---

## 🛠️ Technologies Used

- **C++17 (Multithreading, Forking)**
- **SFML (Simple and Fast Multimedia Library)**
- **System V IPC (Message Queues & Shared Memory)**
- **UNIX Signals & Process Handling**

---


## ⚙️ Setup Instructions

1. **Install SFML**
   ```bash
   sudo apt-get install libsfml-dev
2. **Compile the Project**
   ```bash
   g++ AirControlX.cpp -o AirControlX -lsfml-graphics -lsfml-window -lsfml-system -pthread
3. **Run the Simulation**
   ```bash
   ./AirControlX
4. **To Stop**
   Press Ctrl+C to safely terminate all processes and clean up shared memory.

---

   ## Project Structure
   AirControlX/
├── AirControlX.cpp         # Main source file (includes graphics, logic, and IPC)
├── README.md
└── arial.ttf       # Font file for dashboard rendering

---

## 📌 Notes
- The simulation is time-bound to 5 minutes (adjustable via SIM_TIME macro).
- Shared memory and message queues are cleared on exit.
- All flights are simulated with randomized behavior for realism.
