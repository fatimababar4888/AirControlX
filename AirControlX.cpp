// Aliza Rashid & Fatima Babar
// 22i-1553 & 22I-1565
// CS-E
// Project Module 3


#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <iostream>
#include <cstdlib>
#include <iomanip>
#include <cstring>
#include <ctime>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/msg.h>
#include <signal.h>
#include <fcntl.h>
#include <string>
#include <vector>

using namespace std;

//simulation wali cheezein
#define SIM_TIME 300         //5 min=300secs
#define MAX_FLIGHTS 20
#define MAX_QUEUE 10
#define NUM_RUNWAYS 3
#define MAX_AVNS 50
#define MAX_VIOLATIONS 20
#define SHM_KEY 12345
#define MSG_KEY 54321
#define AVN_MSG_KEY 65432
#define PAYMENT_MSG_KEY 76543

//mutex aur cond variables wali cheezein
mutex runway_locks[NUM_RUNWAYS]; 
mutex queue_lock;  //queue handle krne ke liye lock
mutex avn_lock;   //avn handle krne ke liye lock
condition_variable runway_available[NUM_RUNWAYS];   //runway availability signal krne ke liye

//flight type, direction aur runway define kar rahe hein
enum class AircraftType { COMMERCIAL, CARGO, EMERGENCY };
enum class Direction { NORTH, SOUTH, EAST, WEST };
enum class Runway { RWY_A, RWY_B, RWY_C };
enum class FlightStatus 
{ 
    WAITING, HOLDING, APPROACH, LANDING, TAXI, AT_GATE, 
    GATE, TAKEOFF_ROLL, CLIMB, CRUISING, COMPLETED
};

//flight ka structure n shi
struct Flight
{
    string airline;
    string flightID;
    AircraftType type;
    Direction dir;
    Runway assigned_runway;
    bool isEmergency;
    bool cargoViolation;
    bool speedViolation;
    bool groundFault;
    bool avnIssued;
    FlightStatus status;
    time_t entryTime;
    int fuelStatus;
    int priority;
    int speed;
    thread::id thread_id;
};

//AVN ka structure n shi
struct AVN
{
    string avnID;
    string flightID;
    string airline;
    AircraftType type;
    string reason;
    int fine;
    time_t issueTime;
    time_t dueDate;  // due date (3 days from issuance)
    int paymentStatus;   //0=unpaid, 1=paid, 2=overdue
};

// Message structure for IPC
struct AVNMessage {
    long mtype;
    AVN avn;
};

// Payment message structure
struct PaymentMessage {
    long mtype;
    char avnID[15];
    char flightID[10];
    int amount;
    int status; // 0=payment pending, 1=payment successful
};

// Shared memory structure
struct SharedData {
    AVN avns[MAX_AVNS];
    int avn_count;
    Flight activeFlights[MAX_FLIGHTS];
    int active_flight_count;
    bool simulation_running;
};

//queue ka structure jo flight pointers store krta hei
struct FlightQueue
{
    vector<Flight*> flights;
    int front, rear;
    int totalCount;
    FlightQueue() : front(0), rear(-1), totalCount(0) {
        flights.resize(MAX_QUEUE, nullptr);
    }
};

//global variables etc
AVN avns[MAX_AVNS];
int avn_count = 0;
FlightQueue arrivalQueue;
FlightQueue departureQueue;
FlightQueue emergencyQueue;
bool simulation_running = true;
int active_flights = 0;
int shmid;
SharedData *shared_data;
int msgid, avn_msgid, payment_msgid;

//function ke prototypes
void initSimulation();
void runwayController();
void flightScheduler();
void simulateFlight(Flight* f);
void calculatePriority(Flight* f);
void assignRunway(Flight* f);
void updateFlightStatus(Flight* f, FlightStatus newStatus);
void issueAVN(Flight* f, const string& reason);
void printDashboard(time_t elapsed);
string statusToString(FlightStatus status);
bool checkSpeedViolation(Flight* f, const string& phase, int speed, int min, int max);
void atcsControllerProcess();
void avnGeneratorProcess();
void airlinePortalProcess();
void stripePayProcess();
void cleanup();

//helper functions
//enum values ko readable string mein convert karne ke functions
string runwayToStr(Runway rw) 
{
    static const string strings[] = {"RWY-A", "RWY-B", "RWY-C"};
    return strings[static_cast<int>(rw)];
}

string dirToStr(Direction d)
{
    static const string strings[] = {"North", "South", "East", "West"};
    return strings[static_cast<int>(d)];
}

string typeToStr(AircraftType t) 
{
    static const string strings[] = {"Commercial", "Cargo", "Emergency"};
    return strings[static_cast<int>(t)];
}

string statusToString(FlightStatus status) 
{
    static const string strings[] = 
    {
        "Waiting", "Holding", "Approach", "Landing", "Taxi", "At Gate", 
        "Gate", "Takeoff Roll", "Climb", "Cruising", "Completed"
    };
    return strings[static_cast<int>(status)];
}

int randomInRange(int min, int max) 
{
    return min + rand() % (max - min + 1);
}

//simulation components ko initialise krna hei

void initSimulation() {
    // Initialize mutexes and condition variables (automatic in C++)
    
    // Initialize queues
    arrivalQueue.front = departureQueue.front = emergencyQueue.front = 0;
    arrivalQueue.rear = departureQueue.rear = emergencyQueue.rear = -1;
    arrivalQueue.totalCount = departureQueue.totalCount = emergencyQueue.totalCount = 0;

    // Create shared memory - first try to remove any existing segment
    shmid = shmget(SHM_KEY, sizeof(SharedData), IPC_CREAT | IPC_EXCL | 0666);
    if (shmid == -1) {
        // If segment exists, remove it and try again
        if (errno == EEXIST) {
            int temp_shmid = shmget(SHM_KEY, sizeof(SharedData), 0666);
            if (temp_shmid != -1) {
                shmctl(temp_shmid, IPC_RMID, NULL);
            }
            shmid = shmget(SHM_KEY, sizeof(SharedData), IPC_CREAT | 0666);
        }
        if (shmid == -1) {
            perror("shmget failed");
            exit(1);
        }
    }

    // Attach shared memory
    shared_data = (SharedData *)shmat(shmid, NULL, 0);
    if (shared_data == (void *)-1) {
        perror("shmat failed");
        exit(1);
    }

    // Initialize shared memory
    memset(shared_data, 0, sizeof(SharedData)); // Clear the shared memory
    shared_data->avn_count = 0;
    shared_data->active_flight_count = 0;
    shared_data->simulation_running = true;

    // Create message queues - remove existing ones first
    msgctl(msgget(MSG_KEY, 0666), IPC_RMID, NULL);
    msgid = msgget(MSG_KEY, IPC_CREAT | 0666);
    if (msgid == -1) {
        perror("msgget failed");
        exit(1);
    }

    msgctl(msgget(AVN_MSG_KEY, 0666), IPC_RMID, NULL);
    avn_msgid = msgget(AVN_MSG_KEY, IPC_CREAT | 0666);
    if (avn_msgid == -1) {
        perror("avn msgget failed");
        exit(1);
    }

    msgctl(msgget(PAYMENT_MSG_KEY, 0666), IPC_RMID, NULL);
    payment_msgid = msgget(PAYMENT_MSG_KEY, IPC_CREAT | 0666);
    if (payment_msgid == -1) {
        perror("payment msgget failed");
        exit(1);
    }
}

//flight ko queue mein add karna
void enqueue(FlightQueue* q, Flight* f) 
{
    lock_guard<mutex> lock(queue_lock);
    
    if (q->totalCount == MAX_QUEUE) 
    {
        cout << "[QUEUE] " << (q == &emergencyQueue ? "Emergency" : 
                            q == &arrivalQueue ? "Arrival" : "Departure") 
             << " queue is full. Cannot add flight " << f->flightID << endl;
        return;
    }
    
    q->rear = (q->rear + 1) % MAX_QUEUE;
    q->flights[q->rear] = f;
    q->totalCount++;
    active_flights++;
    
    // Add to shared memory
    if (shared_data->active_flight_count < MAX_FLIGHTS) {
        shared_data->activeFlights[shared_data->active_flight_count++] = *f;
    }
    
    updateFlightStatus(f, FlightStatus::WAITING);
    
    cout << "[QUEUE] Added " << f->flightID << " (" << f->airline << ") to " 
         << (q == &emergencyQueue ? "emergency" : 
             q == &arrivalQueue ? "arrival" : "departure") 
         << " queue | Fuel: " << f->fuelStatus << "% | Priority: " << f->priority << endl;
}

//flight ko queue se nikalna
Flight* dequeue(FlightQueue* q) 
{
    lock_guard<mutex> lock(queue_lock);
    
    if (q->totalCount == 0) return nullptr;
    
    Flight* f = q->flights[q->front];
    q->front = (q->front + 1) % MAX_QUEUE;
    q->totalCount--;

    return f;
}

void calculatePriority(Flight* f)
{
    int base_priority = 0;
    
    //base priority by type
    if (f->type == AircraftType::EMERGENCY) base_priority = 10;
    else if (f->type == AircraftType::CARGO) base_priority = 5;
    else base_priority = 3; //commercial
    
    //adjust for fuel status
    int fuel_priority = 0;
    if (f->fuelStatus < 10) fuel_priority = 5;  //critical fuel, highest priority
    else if (f->fuelStatus < 20) fuel_priority = 3;
    else if (f->fuelStatus < 40) fuel_priority = 1;
    
    //emergency statusFlag override
    int emergency_priority = f->isEmergency ? 10 : 0;
    
    //calculate final priority (take the highest value)
    f->priority = base_priority;
    if (fuel_priority > f->priority) f->priority = fuel_priority;
    if (emergency_priority > f->priority) f->priority = emergency_priority;
    
    if (f->priority == 10) f->isEmergency = true; // If priority is maximum, mark as emergency
}

void assignRunway(Flight* f) 
{
    // emergency flights get RWY-C
    if (f->priority >= 8 || f->isEmergency) {
        f->assigned_runway = Runway::RWY_C;
        return;
    }
    
    // cargo flights must use RWY-C
    if (f->type == AircraftType::CARGO) {
        f->assigned_runway = Runway::RWY_C;
        return;
    }
    
    //regular commercial flights
    if (f->dir == Direction::NORTH || f->dir == Direction::SOUTH) {
        f->assigned_runway = Runway::RWY_A;  // North/South for arrivals
    } else {
        f->assigned_runway = Runway::RWY_B;  // East/West for departures
    }
}

bool checkSpeedViolation(Flight* f, const string& phase, int speed, int min, int max) {
    if (speed >= min && speed <= max) {
        return false;
    }
    
    cout << "[VIOLATION] " << f->flightID << " (" << f->airline << ") Speed Violation in " 
         << phase << ": " << speed << " km/h (Range: " << min << "-" << max << " km/h)" << endl;
    
    if (f->avnIssued) {
        return true;
    }
    
    string reason = "Speed violation in " + phase + " phase (" + to_string(speed) + " km/h)";
    issueAVN(f, reason);
    
    return true;
}

//agar koi violation hou tou avn assign krna
void issueAVN(Flight* f, const string& reason) 
{
    lock_guard<mutex> lock(avn_lock);
    
    if (avn_count == MAX_AVNS) {
        return;
    }
    
    AVN newAvn;
    
    //generate AVN ID
    newAvn.avnID = "AVN-" + string(4 - to_string(avn_count + 1).length(), '0') + to_string(avn_count + 1);
    
    newAvn.flightID = f->flightID;
    newAvn.airline = f->airline;
    newAvn.type = f->type;
    newAvn.reason = reason;
    
    //calculate fine
    newAvn.fine = (f->type == AircraftType::COMMERCIAL) ? 500000 : 700000;
    int service_fee = static_cast<int>(0.15 * newAvn.fine); // 15% service fee
    newAvn.fine += service_fee;
    
    newAvn.issueTime = time(nullptr);
    newAvn.dueDate = newAvn.issueTime + (3 * 24 * 60 * 60); // 3 days from issuance
    newAvn.paymentStatus = 0; // Unpaid
    
    avns[avn_count] = newAvn;
    f->avnIssued = true;
    
    // Add to shared memory
    if (shared_data->avn_count < MAX_AVNS) {
        shared_data->avns[shared_data->avn_count] = newAvn;
        shared_data->avn_count++;
    }
    
    // Send AVN to AVN Generator Process
    AVNMessage messageData;
    messageData.mtype = 1;
    messageData.avn = newAvn;
    
    if (msgsnd(avn_msgid, &messageData, sizeof(AVNMessage) - sizeof(long), 0) == -1) {
        perror("msgsnd failed");
    }
    
    cout << "[AVN] " << newAvn.avnID << " issued to " << f->flightID << " (" << f->airline 
         << ") | Reason: " << reason << " | Fine: PKR " << newAvn.fine << " (includes 15% service fee)" << endl;
    
    avn_count++;
}

void updateFlightStatus(Flight* f, FlightStatus newStatus)
{
    f->status = newStatus;
    
    //print status update with color based on status
    const char* color_code;
    switch(newStatus) {
        case FlightStatus::WAITING:    color_code = "\033[0;36m"; break;      //Cyan
        case FlightStatus::HOLDING:    color_code = "\033[0;33m"; break;      //Yellow
        case FlightStatus::APPROACH:   color_code = "\033[0;35m"; break;      //Purple
        case FlightStatus::LANDING:    color_code = "\033[0;32m"; break;      //Green
        case FlightStatus::TAKEOFF_ROLL: color_code = "\033[0;34m"; break;    //Blue
        case FlightStatus::COMPLETED:  color_code = "\033[1;32m"; break;      //Green
        default:         color_code = "\033[0m";     break;     //Default
    }
    
    cout << color_code << "[STATUS] " << f->flightID << " is now " 
         << statusToString(newStatus) << "\033[0m" << endl;
    
    if (newStatus != FlightStatus::COMPLETED)
    {
        return;
    }
    
    active_flights--;
    
    // Update shared memory
    shared_data->active_flight_count = active_flights;
}

//har flight ke liye thread chalakar simulate karna
void simulateFlight(Flight* f)
{
    Runway rwy = f->assigned_runway;
    
    //lock the assigned runway
    unique_lock<mutex> lock(runway_locks[static_cast<int>(rwy)]);
    
    cout << "\n\033[1;36m[RUNWAY]\033[0m " << runwayToStr(rwy) << " assigned to " 
         << f->flightID << " (" << f->airline << ") for " 
         << (f->dir != Direction::NORTH && f->dir != Direction::SOUTH ? "takeoff" : "landing") 
         << " | Fuel: " << f->fuelStatus << "%" << endl;
    
    //simulate flight phases
    if (f->dir == Direction::NORTH || f->dir == Direction::SOUTH) 
    {
        //landing sequence
        struct Phase {
            string name;
            int minSpeed;
            int maxSpeed;
            FlightStatus status;
        };
        vector<Phase> phases = {
            {"Holding", 400, 600, FlightStatus::HOLDING}, 
            {"Approach", 240, 290, FlightStatus::APPROACH}, 
            {"Landing", 30, 240, FlightStatus::LANDING}, 
            {"Taxi", 15, 30, FlightStatus::TAXI}, 
            {"At Gate", 0, 5, FlightStatus::AT_GATE}
        };
        
        //process each phase
        for (const auto& phase : phases) {
            updateFlightStatus(f, phase.status);
            
            //simulate speed iss phase ke liye 
            int speed = randomInRange(phase.minSpeed - 30, phase.maxSpeed + 30);
            f->speed = speed;
            
            cout << "[FLIGHT] " << f->flightID << " in " << phase.name 
                 << " phase | Speed: " << speed << " km/h (Range: " 
                 << phase.minSpeed << "-" << phase.maxSpeed << " km/h)" << endl;
            
            //check 4 speed ki violation
            checkSpeedViolation(f, phase.name, speed, phase.minSpeed, phase.maxSpeed);
            
            //simulate fuel ki consumption (only 4 demonstration)
            if (f->fuelStatus <= 5) 
            {
                f->fuelStatus = 0;
            }
            else 
            {
                f->fuelStatus -= randomInRange(1, 3);
            }
            
            //simulate phase ki duration
            this_thread::sleep_for(chrono::microseconds(800000)); //0.8 seconds per phase
        }
        
    } else 
    {
        //takeoff wala sequence
        struct Phase {
            string name;
            int minSpeed;
            int maxSpeed;
            FlightStatus status;
        };
        vector<Phase> phases = {
            {"At Gate", 0, 5, FlightStatus::GATE}, 
            {"Taxi", 15, 30, FlightStatus::TAXI}, 
            {"Takeoff Roll", 0, 290, FlightStatus::TAKEOFF_ROLL}, 
            {"Climb", 250, 463, FlightStatus::CLIMB}, 
            {"Cruise", 800, 900, FlightStatus::CRUISING}
        };
        
        //process har phase
        for (size_t idx = 0; idx < phases.size(); ++idx) {
            updateFlightStatus(f, phases[idx].status);
            
            //simulate speed iss phase ke liye
            int speed = randomInRange(phases[idx].minSpeed - 30, phases[idx].maxSpeed + 30);
            f->speed = speed;
            
            cout << "[FLIGHT] " << f->flightID << " in " << phases[idx].name 
                 << " phase | Speed: " << speed << " km/h (Range: " 
                 << phases[idx].minSpeed << "-" << phases[idx].maxSpeed << " km/h)" << endl;
            
            //speed violation check krni hei
            checkSpeedViolation(f, phases[idx].name, speed, phases[idx].minSpeed, phases[idx].maxSpeed);
            
            //simulate fuel ki consumption (only 4 demonstration)
            if (f->fuelStatus > 5) 
            {
                f->fuelStatus -= randomInRange(1, 2);
                if (f->fuelStatus < 0) f->fuelStatus = 0;
            }
            
            //East/West flights ke liye dekhni hei ground fault during taxi
            if (idx == 1 && (f->dir == Direction::EAST || f->dir == Direction::WEST) && randomInRange(1, 10) <= 2)
            {
                f->groundFault = true;
                cout << "[FAULT] Ground fault detected for " << f->flightID << " during taxi!" << endl;
                issueAVN(f, "Ground fault occurred (brake/hydraulic)");
                
                //baqi remaining phases ko skip kr ke directly completion pe jana hei
                break;
            }
            
            //simulate phase duration
            this_thread::sleep_for(chrono::microseconds(800000)); //0.8 seconds per phase
        }
    }
    
    updateFlightStatus(f, FlightStatus::COMPLETED);
    cout << "\033[1;32m[COMPLETE]\033[0m Flight " << f->flightID << " finished" << endl;
    
    //release runway
    lock.unlock();
    runway_available[static_cast<int>(rwy)].notify_one();
}

void printDashboard(time_t elapsed) 
{
    //flights by airline totalCount krta hei
    int airline_counts[6] = {0}; 
    int active_violations = 0;
    
    lock_guard<mutex> lock(queue_lock);
    
    //totalCount emergency queue
    for (int idx = 0; idx < emergencyQueue.totalCount; ++idx) {
        Flight* f = emergencyQueue.flights[(emergencyQueue.front + idx) % MAX_QUEUE];

        if (f->airline == "PIA") airline_counts[0]++;
        else if (f->airline == "AirBlue") airline_counts[1]++;
        else if (f->airline == "FedEx") airline_counts[2]++;
        else if (f->airline == "PAF") airline_counts[3]++;
        else if (f->airline == "BlueDart") airline_counts[4]++;
        else if (f->airline == "AghaKhan") airline_counts[5]++;
        
        if (f->avnIssued) {
            active_violations++;
        }
    }
    
    //totalCount arrival queue
    for (int idx = 0; idx < arrivalQueue.totalCount; ++idx) {
        Flight* f = arrivalQueue.flights[(arrivalQueue.front + idx) % MAX_QUEUE];

        if (f->airline == "PIA") airline_counts[0]++;
        else if (f->airline == "AirBlue") airline_counts[1]++;
        else if (f->airline == "FedEx") airline_counts[2]++;
        else if (f->airline == "PAF") airline_counts[3]++;
        else if (f->airline == "BlueDart") airline_counts[4]++;
        else if (f->airline == "AghaKhan") airline_counts[5]++;
        
        if (f->avnIssued) {
            active_violations++;
        }
    }
    
    //totalCount departure queue
    for (int idx = 0; idx < departureQueue.totalCount; ++idx) {
        Flight* f = departureQueue.flights[(departureQueue.front + idx) % MAX_QUEUE];
        
        if (f->airline == "PIA") airline_counts[0]++;
        else if (f->airline == "AirBlue") airline_counts[1]++;
        else if (f->airline == "FedEx") airline_counts[2]++;
        else if (f->airline == "PAF") airline_counts[3]++;
        else if (f->airline == "BlueDart") airline_counts[4]++;
        else if (f->airline == "AghaKhan") airline_counts[5]++;
        
        if (f->avnIssued) {
            active_violations++;
        }
    }
    
    cout << "\n\033[1;36m=============== AirControlX Dashboard = ==============\033[0m" << endl;
    cout << "Simulation Time: " << elapsed / 60 << ":" << setw(2) << setfill('0') << elapsed % 60 << endl;
    cout << "Active Flights: " << active_flights << endl;

    cout << "Queue Status: Emergency: " << emergencyQueue.totalCount 
         << " | Arrival: " << arrivalQueue.totalCount 
         << " | Departure: " << departureQueue.totalCount << endl;

    cout << "AVNs Issued: " << avn_count << endl;
    cout << "Active Violations: " << active_violations << endl;
    cout << "Airline Activity:" << endl;
    cout << "  - PIA: " << airline_counts[0] << " flights" << endl;
    cout << "  - AirBlue: " << airline_counts[1] << " flights" << endl;
    cout << "  - FedEx: " << airline_counts[2] << " flights" << endl;
    cout << "  - PAF: " << airline_counts[3] << " flights" << endl;
    cout << "  - BlueDart: " << airline_counts[4] << " flights" << endl;
    cout << "  - AghaKhan: " << airline_counts[5] << " flights" << endl;

    cout << "\033[1;36m= ===================================================\033[0m" << endl;
}

//controller thread: queues ko monitor karna aur flights ko process karna
void runwayController() 
{
    cout << "[CONTROLLER] Runway controller started" << endl;
    
    while (simulation_running) {
        // check emergency queue sab se pehle 
        Flight* nextFlight = dequeue(&emergencyQueue);
        
        //phir check arrival queue
        if (nextFlight == nullptr) 
        {
            nextFlight = dequeue(&arrivalQueue);
        }
        
        //akhir mein check departure queue
        if (nextFlight == nullptr)
        {
            nextFlight = dequeue(&departureQueue);
        }
        
        if (nextFlight != nullptr) 
        {
            //iss flight ko process krna
            cout << "\n[CONTROLLER] Processing flight " << nextFlight->flightID 
                 << " (" << nextFlight->airline << ")" << endl;
            
            // Create thread iss flight ke liye
            thread flightThread(simulateFlight, nextFlight);
            flightThread.detach();
        } else
        {
            //no flights to process, sleep thori dair ke liye
            this_thread::sleep_for(chrono::microseconds(500000)); // 0.5 seconds
        }
    }
    
    cout << "[CONTROLLER] Runway controller stopped" << endl;
}

//naye flights generate karne aur queue mein daalne ka kaam karta hai
void flightScheduler() 
{
    cout << "[SCHEDULER] Flight scheduler started" << endl;
    
    time_t start = time(nullptr);
    int flightCounter = 0;
    
    //flight ke pre-defined templates banaye hain (airline, type, direction)
    struct FlightTemplate {
        string airline;
        AircraftType type;
        Direction dir;
    };
    const vector<FlightTemplate> flightTemplates = {
        {"PIA", AircraftType::COMMERCIAL, Direction::NORTH}, //PIA - International Arrival
        {"PIA", AircraftType::COMMERCIAL, Direction::WEST}, //PIA - Domestic Departure
        {"AirBlue", AircraftType::COMMERCIAL, Direction::SOUTH}, //AirBlue - Domestic Arrival
        {"AirBlue", AircraftType::COMMERCIAL, Direction::EAST}, //AirBlue - International Departure
        {"FedEx", AircraftType::CARGO, Direction::WEST}, //FedEx - Domestic Cargo Departure
        {"FedEx", AircraftType::CARGO, Direction::EAST}, //FedEx - International Cargo Departure
        {"PAF", AircraftType::EMERGENCY, Direction::EAST}, //Pakistan Air Force - Military
        {"BlueDart", AircraftType::CARGO, Direction::SOUTH}, //BlueDart - Domestic Cargo Arrival
        {"BlueDart", AircraftType::CARGO, Direction::WEST}, //BlueDart - Domestic Cargo Departure
        {"AghaKhan", AircraftType::EMERGENCY, Direction::NORTH} //AghaKhan - Medical Emergency
    };
    
    //jab tak simulation ka waqt nahi khatam hota, loop chalate rahe
    while (simulation_running) {
        time_t elapsed = time(nullptr) - start;
        if (elapsed >= SIM_TIME) 
        {
            simulation_running = false;
            shared_data->simulation_running = false;
            break;
        }
        
        //har 7 seconds pe dashboard update ho
        if (elapsed % 7 == 0) 
        {
            printDashboard(elapsed);
        }
        
        //probability check karta hai new flight generate karne ke liye
        if (rand() % 100 < 60 && active_flights < MAX_FLIGHTS)   //60% chance
        {
            //random flight template choose krta hai
            int tplIndex = rand() % flightTemplates.size();
            
            //flight Id ki generation krta hai
            string flightID;
            
            //Is airline cargo hain?
            if (flightTemplates[tplIndex].airline == "FedEx" || 
                flightTemplates[tplIndex].airline == "BlueDart")
            {
                //Cargo flight IDs: C-[0-9]{3}
                flightID = "C-" + string(3 - to_string(100 + (flightCounter++ % 900)).length(), '0') 
                         + to_string(100 + (flightCounter % 900));
            }
            //Is airline emergency?
            else if (flightTemplates[tplIndex].airline == "PAF" || 
                     flightTemplates[tplIndex].airline == "AghaKhan")
            {
                //Emergency flight IDs: E-[0-9]{3}
                flightID = "E-" + string(3 - to_string(100 + (flightCounter++ % 900)).length(), '0') 
                         + to_string(100 + (flightCounter % 900));
            }
            else
            {
                //Commercial flight IDs: [A-Z]{2}[0-9]{3}
                //PIA: PK, AirBlue: AB
                string prefix = "PK";
                if (flightTemplates[tplIndex].airline == "AirBlue")
                {
                    prefix = "AB";
                }
                flightID = prefix + string(3 - to_string(100 + (flightCounter++ % 900)).length(), '0') 
                         + to_string(100 + (flightCounter % 900));
            }
            
            //Create a new flight
            Flight* newFlight = new Flight;
            newFlight->airline = flightTemplates[tplIndex].airline;
            newFlight->flightID = flightID;
            newFlight->type = flightTemplates[tplIndex].type;
            newFlight->dir = flightTemplates[tplIndex].dir;
            newFlight->isEmergency = false;
            newFlight->speedViolation = false;
            newFlight->cargoViolation = false;
            newFlight->groundFault = false;
            newFlight->avnIssued = false;
            newFlight->status = FlightStatus::WAITING;
            newFlight->entryTime = time(nullptr);
            newFlight->fuelStatus = randomInRange(30, 100);
            newFlight->speed = 0;
            
            //introduce low fuel scenario
            if (rand() % 100 < 15)  //15% chance
            {
                newFlight->fuelStatus = randomInRange(5, 20);
                cout << "[EMERGENCY] " << newFlight->flightID << " has low fuel: " 
                     << newFlight->fuelStatus << "%" << endl;
            }
            
            //introduce emergency scenario
            //Emergency flights (PAF, AghaKhan) are always emergencies
            if (newFlight->airline == "PAF" || newFlight->airline == "AghaKhan")
            {
                newFlight->isEmergency = true;
            }
            //other airlines have a small chance
            else if (rand() % 100 < 8)  //8% chance
            {
                newFlight->isEmergency = true;
                cout << "[EMERGENCY] " << newFlight->flightID << " has declared an emergency!" << endl;
            }
            
            //calculate priority based on type, fuel, emergency status
            calculatePriority(newFlight);
            
            //assign runway
            assignRunway(newFlight);
            
            //enqueue to appropriate queue
            if (newFlight->isEmergency || newFlight->priority >= 8)
            {
                enqueue(&emergencyQueue, newFlight);
            }
            else if (newFlight->dir == Direction::NORTH || newFlight->dir == Direction::SOUTH)
            {
                enqueue(&arrivalQueue, newFlight);
            }
            else
            {
                enqueue(&departureQueue, newFlight);
            }
        }
        
        //Sleep random for each cycle
        this_thread::sleep_for(chrono::microseconds(randomInRange(1000000, 1500000))); //1-1.5 seconds
    }
    
    cout << "[SCHEDULER] Flight scheduler stopped" << endl;
}

void atcsControllerProcess() {
    cout << "[ATCS] ATCS Controller process started (PID: " << getpid() << ")" << endl;
    
    // This process monitors violations and updates status in shared memory
    while (shared_data->simulation_running) {
        sleep(1);
        
        // Check for violations in active flights
        int violation_count = 0;
        for (int idx = 0; idx < shared_data->active_flight_count; idx++) {
            Flight* f = &shared_data->activeFlights[idx];
            if (f->avnIssued) {
                violation_count++;
            }
        }
        
        // Display analytics periodically
        static time_t last_update = 0;
        time_t now = time(nullptr);
        if (now - last_update >= 10) {  // Update every 10 seconds
            cout << "\n[ATCS Analytics] Active Violations: " << violation_count << endl;
            
            // Display aircraft with active violations
            if (violation_count > 0) {
                cout << "Aircraft with Active Violations:" << endl;
                for (int idx = 0; idx < shared_data->active_flight_count; idx++) {
                    Flight* f = &shared_data->activeFlights[idx];
                    if (f->avnIssued) {
                        cout << "  - " << f->flightID << " (" << f->airline << "): ";
                        
                        if (f->speedViolation) cout << "Speed ";
                        if (f->cargoViolation) cout << "Cargo ";
                        if (f->groundFault) cout << "Ground ";
                        
                        cout << endl;
                    }
                }
            }
            
            last_update = now;
        }
    }
    
    cout << "[ATCS] ATCS Controller process exiting" << endl;
    exit(0);
}

void avnGeneratorProcess() {
    cout << "[AVN] AVN Generator process started (PID: " << getpid() << ")" << endl;
    
    AVNMessage messageData;
    
    while (shared_data->simulation_running) {
        // Receive AVN from main process
        if (msgrcv(avn_msgid, &messageData, sizeof(AVNMessage) - sizeof(long), 1, IPC_NOWAIT) != -1) {
            AVN* avn = &messageData.avn;
            
            cout << "\n[AVN Generator] New violation received:" << endl;
            cout << "  AVN ID: " << avn->avnID << endl;
            cout << "  Flight: " << avn->flightID << " (" << avn->airline << ")" << endl;
            cout << "  Type: " << typeToStr(avn->type) << endl;
            cout << "  Reason: " << avn->reason << endl;
            cout << "  Fine: PKR " << avn->fine << " (includes 15% service)" << endl;
            cout << "  Due Date: " << ctime(&avn->dueDate);
            
            // Forward to Airline Portal by updating shared memory
            // (In a real system, this would use more sophisticated IPC)
            for (int idx = 0; idx < shared_data->avn_count; idx++) {
                if (shared_data->avns[idx].avnID == avn->avnID) {
                    // Already exists, update it
                    shared_data->avns[idx] = *avn;
                    break;
                }
            }
        }
        
        // Check for payment confirmations from StripePay
        PaymentMessage payment;
        if (msgrcv(payment_msgid, &payment, sizeof(PaymentMessage) - sizeof(long), 2, IPC_NOWAIT) != -1) {
            if (payment.status == 1) {
                cout << "\n[AVN Generator] Payment confirmation received for " << payment.avnID << endl;
                
                // Update payment status in shared memory
                for (int idx = 0; idx < shared_data->avn_count; idx++) {
                    if (shared_data->avns[idx].avnID == payment.avnID) {
                        shared_data->avns[idx].paymentStatus = 1; // Paid
                        
                        cout << "[AVN Generator] Updated " << payment.avnID << " payment status to PAID" << endl;
                        
                        // Forward confirmation to Airline Portal
                        // (In a real system, this would use more sophisticated IPC)
                        break;
                    }
                }
            }
        }
        
        this_thread::sleep_for(chrono::microseconds(500000)); // 0.5 seconds
    }
    
    cout << "[AVN] AVN Generator process exiting" << endl;
    exit(0);
}

void airlinePortalProcess() {
    cout << "[Airline] Airline Portal process started (PID: " << getpid() << ")" << endl;
    
    while (shared_data->simulation_running) {
        // Display periodically
        static time_t last_update = 0;
        time_t now = time(nullptr);
        
        if (now - last_update >= 5) {  // Update every 5 seconds
            cout << "\n\033[1;35m=============== Airline Portal ===============\033[0m" << endl;
            cout << "Active AVNs:" << endl;
            
            for (int idx = 0; idx < shared_data->avn_count; idx++) {
                AVN* avn = &shared_data->avns[idx];
                
                cout << "  " << avn->avnID << " | Flight: " << avn->flightID 
                     << " | Airline: " << avn->airline << " | Fine: PKR " << avn->fine 
                     << " | Status: " 
                     << (avn->paymentStatus == 0 ? "UNPAID" : 
                         avn->paymentStatus == 1 ? "PAID" : "OVERDUE") << endl;
            }
            
            cout << "\033[1;35m===========================================\033[0m" << endl;
            last_update = now;
        }
        
        // Simulate airline admin making payments (random)
        if (shared_data->avn_count > 0 && rand() % 100 < 20) {  // 20% chance
            int idx = rand() % shared_data->avn_count;
            AVN* avn = &shared_data->avns[idx];
            
            if (avn->paymentStatus == 0) {  // If unpaid
                cout << "\n[Airline Portal] Initiating payment for " << avn->avnID << "..." << endl;
                
                // Send payment request to StripePay
                PaymentMessage payment;
                payment.mtype = 1;
                strncpy(payment.avnID, avn->avnID.c_str(), sizeof(payment.avnID));
                strncpy(payment.flightID, avn->flightID.c_str(), sizeof(payment.flightID));
                payment.amount = avn->fine;
                payment.status = 0;  // Pending
                
                if (msgsnd(payment_msgid, &payment, sizeof(PaymentMessage) - sizeof(long), 0) == -1) {
                    perror("msgsnd failed");
                } else {
                    cout << "[Airline Portal] Payment request sent to StripePay" << endl;
                }
            }
        }
        
        this_thread::sleep_for(chrono::microseconds(1000000)); // 1 second
    }
    
    cout << "[Airline] Airline Portal process exiting" << endl;
    exit(0);
}

void stripePayProcess() {
    cout << "[StripePay] StripePay process started (PID: " << getpid() << ")" << endl;
    
    while (shared_data->simulation_running) {
        // Receive payment requests from Airline Portal
        PaymentMessage payment;
        if (msgrcv(payment_msgid, &payment, sizeof(PaymentMessage) - sizeof(long), 1, IPC_NOWAIT) != -1) {
            cout << "\n[StripePay] Payment request received:" << endl;
            cout << "  AVN ID: " << payment.avnID << endl;
            cout << "  Flight: " << payment.flightID << endl;
            cout << "  Amount: PKR " << payment.amount << endl;
            
            // Simulate payment processing
            this_thread::sleep_for(chrono::microseconds(2000000));  // 2 seconds processing time
            
            // 90% success rate
            if (rand() % 100 < 90) {
                cout << "[StripePay] Payment successful for " << payment.avnID << endl;
                payment.status = 1;  // Successful
            } else {
                cout << "[StripePay] Payment failed for " << payment.avnID << endl;
                payment.status = 0;  // Failed
            }
            
            // Send response back to AVN Generator
            payment.mtype = 2;  // Response message type
            if (msgsnd(payment_msgid, &payment, sizeof(PaymentMessage) - sizeof(long), 0) == -1) {
                perror("msgsnd failed");
            }
        }
        
        this_thread::sleep_for(chrono::microseconds(500000)); // 0.5 seconds
    }
    
    cout << "[StripePay] StripePay process exiting" << endl;
    exit(0);
}

void cleanup() {
    cout << "\nCleaning up resources..." << endl;
    
    // Destroy mutexes and condition variables
    // In C++, mutex and condition_variable are automatically destroyed
    
    // Detach and remove shared memory
    shmdt(shared_data);
    shmctl(shmid, IPC_RMID, NULL);
    
    // Remove message queues
    msgctl(msgid, IPC_RMID, NULL);
    msgctl(avn_msgid, IPC_RMID, NULL);
    msgctl(payment_msgid, IPC_RMID, NULL);
    
    cout << "Simulation terminated." << endl;
}

// Signal handler for clean shutdown
void sigint_handler(int sig) {
    cout << "\nReceived shutdown signal. Terminating simulation..." << endl;
    simulation_running = false;
    shared_data->simulation_running = false;
}

//ADD ALL THIS CODE B4 MAIN

// SFML Graphics Constants
const int WINDOW_WIDTH = 1200;
const int WINDOW_HEIGHT = 800;
const int RUNWAY_LENGTH = 600;
const int RUNWAY_WIDTH = 40;
const int AIRCRAFT_SIZE = 20;

// Colors
const sf::Color RUNWAY_A_COLOR(100, 100, 255);    // Blue for RWY-A
const sf::Color RUNWAY_B_COLOR(255, 100, 100);    // Red for RWY-B
const sf::Color RUNWAY_C_COLOR(100, 255, 100);    // Green for RWY-C
const sf::Color COMMERCIAL_COLOR(255, 255, 0);    // Yellow for commercial
const sf::Color CARGO_COLOR(255, 165, 0);         // Orange for cargo
const sf::Color EMERGENCY_COLOR(255, 0, 0);        // Red for emergency
const sf::Color BACKGROUND_COLOR(50, 50, 50);      // Dark gray background
const sf::Color TEXT_COLOR(255, 255, 255);         // White text

// Runway positions
const sf::Vector2f RWY_A_START(200, 100);
const sf::Vector2f RWY_A_END(200, 700);
const sf::Vector2f RWY_B_START(400, 400);
const sf::Vector2f RWY_B_END(800, 400);
const sf::Vector2f RWY_C_START(600, 100);
const sf::Vector2f RWY_C_END(600, 700);

// Function to get aircraft color based on type
sf::Color getAircraftColor(AircraftType type) {
    switch(type) {
        case AircraftType::COMMERCIAL: return COMMERCIAL_COLOR;
        case AircraftType::CARGO: return CARGO_COLOR;
        case AircraftType::EMERGENCY: return EMERGENCY_COLOR;
        default: return sf::Color::White;
    }
}

// Function to draw runway
void drawRunway(sf::RenderWindow& window, Runway rw, bool active) {
    sf::RectangleShape runway(sf::Vector2f(RUNWAY_LENGTH, RUNWAY_WIDTH));
    runway.setOrigin(RUNWAY_WIDTH/2, RUNWAY_WIDTH/2);
    
    switch(rw) {
        case Runway::RWY_A:
            runway.setPosition(RWY_A_START.x, (RWY_A_START.y + RWY_A_END.y)/2);
            runway.setRotation(90);
            runway.setFillColor(active ? RUNWAY_A_COLOR : sf::Color(RUNWAY_A_COLOR.r/2, RUNWAY_A_COLOR.g/2, RUNWAY_A_COLOR.b/2));
            break;
        case Runway::RWY_B:
            runway.setPosition((RWY_B_START.x + RWY_B_END.x)/2, RWY_B_START.y);
            runway.setRotation(0);
            runway.setFillColor(active ? RUNWAY_B_COLOR : sf::Color(RUNWAY_B_COLOR.r/2, RUNWAY_B_COLOR.g/2, RUNWAY_B_COLOR.b/2));
            break;
        case Runway::RWY_C:
            runway.setPosition(RWY_C_START.x, (RWY_C_START.y + RWY_C_END.y)/2);
            runway.setRotation(90);
            runway.setFillColor(active ? RUNWAY_C_COLOR : sf::Color(RUNWAY_C_COLOR.r/2, RUNWAY_C_COLOR.g/2, RUNWAY_C_COLOR.b/2));
            break;
    }
    
    window.draw(runway);
}

// Function to draw aircraft
void drawAircraft(sf::RenderWindow& window, const Flight& flight, float progress) {
    sf::CircleShape aircraft(AIRCRAFT_SIZE);
    aircraft.setFillColor(getAircraftColor(flight.type));
    aircraft.setOutlineThickness(2);
    aircraft.setOutlineColor(sf::Color::Black);
    
    sf::Vector2f position;
    float rotation = 0;
    
    // Calculate position based on flight status and runway
    switch(flight.assigned_runway) {
        case Runway::RWY_A:
            if (flight.dir == Direction::NORTH || flight.dir == Direction::SOUTH) {
                // Arrival
                position.x = RWY_A_START.x;
                position.y = RWY_A_START.y + (RWY_A_END.y - RWY_A_START.y) * progress;
                rotation = flight.dir == Direction::NORTH ? 180 : 0;
            }
            break;
        case Runway::RWY_B:
            if (flight.dir == Direction::EAST || flight.dir == Direction::WEST) {
                // Departure
                position.x = RWY_B_START.x + (RWY_B_END.x - RWY_B_START.x) * progress;
                position.y = RWY_B_START.y;
                rotation = flight.dir == Direction::EAST ? 0 : 180;
            }
            break;
        case Runway::RWY_C:
            position.x = RWY_C_START.x;
            position.y = RWY_C_START.y + (RWY_C_END.y - RWY_C_START.y) * progress;
            rotation = 90;
            break;
    }
    
    aircraft.setPosition(position);
    aircraft.setRotation(rotation);
    window.draw(aircraft);
    
    // Draw flight ID
    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        std::cerr << "Failed to load font" << std::endl;
        return;
    }
    
    sf::Text text(flight.flightID, font, 12);
    text.setFillColor(TEXT_COLOR);
    text.setPosition(position.x + AIRCRAFT_SIZE, position.y - AIRCRAFT_SIZE);
    window.draw(text);
}

// Function to draw the dashboard
void drawDashboard(sf::RenderWindow& window, const SharedData* sharedData, time_t elapsed) {
    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        std::cerr << "Failed to load font" << std::endl;
        return;
    }
    
    // Background for dashboard
    sf::RectangleShape dashboard(sf::Vector2f(WINDOW_WIDTH, 150));
    dashboard.setFillColor(sf::Color(0, 0, 0, 200));
    dashboard.setPosition(0, WINDOW_HEIGHT - 150);
    window.draw(dashboard);
    
    // Simulation time
    sf::Text timeText("Simulation Time: " + std::to_string(elapsed / 60) + ":" + 
                     (elapsed % 60 < 10 ? "0" : "") + std::to_string(elapsed % 60), font, 18);
    timeText.setFillColor(TEXT_COLOR);
    timeText.setPosition(20, WINDOW_HEIGHT - 140);
    window.draw(timeText);
    
    // Active flights
    sf::Text flightsText("Active Flights: " + std::to_string(sharedData->active_flight_count), font, 18);
    flightsText.setFillColor(TEXT_COLOR);
    flightsText.setPosition(20, WINDOW_HEIGHT - 110);
    window.draw(flightsText);
    
    // AVNs issued
    sf::Text avnText("AVNs Issued: " + std::to_string(sharedData->avn_count), font, 18);
    avnText.setFillColor(TEXT_COLOR);
    avnText.setPosition(20, WINDOW_HEIGHT - 80);
    window.draw(avnText);
    
    // Active violations
    int violations = 0;
    for (int i = 0; i < sharedData->active_flight_count; i++) {
        if (sharedData->activeFlights[i].avnIssued) violations++;
    }
    sf::Text violationsText("Active Violations: " + std::to_string(violations), font, 18);
    violationsText.setFillColor(TEXT_COLOR);
    violationsText.setPosition(20, WINDOW_HEIGHT - 50);
    window.draw(violationsText);
}

// Main graphics rendering function
void renderGraphics() {
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "AirControlX Simulation");
    window.setFramerateLimit(30);
    
    sf::Clock clock;
    
    while (window.isOpen() && shared_data->simulation_running) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }
        
        window.clear(BACKGROUND_COLOR);
        
        // Draw runways
        drawRunway(window, Runway::RWY_A, false);
        drawRunway(window, Runway::RWY_B, false);
        drawRunway(window, Runway::RWY_C, false);
        
        // Draw active flights
        for (int i = 0; i < shared_data->active_flight_count; i++) {
            // Simple progress calculation for demo purposes
            float progress = (i % 100) / 100.0f;
            drawAircraft(window, shared_data->activeFlights[i], progress);
        }
        
        // Draw dashboard
        time_t elapsed = clock.getElapsedTime().asSeconds();
        drawDashboard(window, shared_data, elapsed);
        
        window.display();
    }
}


int main() {
    // Set random seed
    srand(time(nullptr));
    
    // Register signal handler
    signal(SIGINT, sigint_handler);
    
    // Initialize simulation
    cout << "Initializing AirControlX simulation..." << endl;
    initSimulation();
    
    // Create controller and scheduler threads
    thread controller_thread(runwayController);
    thread scheduler_thread(flightScheduler);
    
    // Start graphics thread
    thread graphics_thread(renderGraphics);
    
    // Fork processes for subsystems
    pid_t atcs_pid = fork();
    if (atcs_pid == 0) {
        // Child process for ATCS Controller
        atcsControllerProcess();
        exit(0);
    }
    
    pid_t avn_pid = fork();
    if (avn_pid == 0) {
        // Child process for AVN Generator
        avnGeneratorProcess();
        exit(0);
    }
    
    pid_t airline_pid = fork();
    if (airline_pid == 0) {
        // Child process for Airline Portal
        airlinePortalProcess();
        exit(0);
    }
    
    pid_t stripe_pid = fork();
    if (stripe_pid == 0) {
        // Child process for StripePay
        stripePayProcess();
        exit(0);
    }
    
    cout << "\nAirControlX Simulation Started" << endl;
    cout << "Press Ctrl+C to stop simulation" << endl << endl;
    
    // Wait for threads to complete (simulation time)
    controller_thread.join();
    scheduler_thread.join();
    graphics_thread.join();
    
    // Kill child processes
    kill(atcs_pid, SIGTERM);
    kill(avn_pid, SIGTERM);
    kill(airline_pid, SIGTERM);
    kill(stripe_pid, SIGTERM);
    
    // Wait for all child processes
    while (wait(nullptr) > 0);
    
    // Clean up resources
    cleanup();
    
    return 0;
}
