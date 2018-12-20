#include <iostream>
#include <vector>
#include <deque>
#include <string>
#include <regex>
#include <iomanip>
#include <unordered_map>
#include <queue>
#include <utility>
#include <algorithm>

//Process Control Block Class
class PCB {
public:

  PCB(int PID, float initialBurstEstimate, unsigned int size ) :
      pid_( PID ), tau_previous_( initialBurstEstimate ), process_size_( size ) {
        resetPcbValues();
        tau_next_remaining_ = tau_previous_;
        cpu_usage_ = 0.0;
        number_of_bursts_ = 0;
        tau_next_ = 0;
        t_previous_ = 0;
      }

  inline void updateFileName( std::string name ) { file_name_ = name; }
  inline void updateMemStart( int start ) { mem_start_ = start; }
  inline void updatePhysicalMemStart( int start ) { physical_mem_start_ = start; }
  inline void updateReadOrWrite( char rw ) { read_or_write_ = rw; }
  inline void updateFileLength( int length ) { file_length_ = length; }
  inline void updateCylinder( int cylinder ) { cylinder_ = cylinder; }

  inline void updateBurstEstimateRemaining( float timeUsed ) { tau_next_remaining_ -= timeUsed; }
  inline void updateCPUUsage( float timeUsed ) { cpu_usage_ += timeUsed; }
  inline void updateTPrevious( float timeUsed ) { t_previous_ += timeUsed; }
  inline void resetTPreviousToZero() { t_previous_ = 0; }

  inline float burstEstimateRemaining() { return tau_next_remaining_; }
  inline float averageBurstTime() { return number_of_bursts_ > 0 ? cpu_usage_ / number_of_bursts_ : 0; }
  inline int PID() { return pid_; }
  inline int totalCpuTime() { return cpu_usage_; }
  inline int cylinder() { return cylinder_; }
  inline unsigned int processSize() { return process_size_; }
  inline unsigned int pagesNeeded() { return pages_needed_; }
  inline void updatePagesNeeded( unsigned int pages ) { pages_needed_ = pages; }

  void resetPcbValues() {
    file_name_ = "";
    mem_start_ = 0;
    read_or_write_ = 'w';
    file_length_ = 0;
    cylinder_ = 0;
  }

  void printPcbContents( std::string qType ) {
    std::cout << std::left;
    std::cout << std::setw(12) << pid_;
    std::cout << std::setw(12) << cpu_usage_;
    std::cout << std::setw(12) << averageBurstTime();
    std::cout << std::setw(12) << tau_next_;
    std::cout << std::setw(12) << tau_next_remaining_;

    if( qType == "f" || qType == "p" || qType == "d" ) {
      std::cout << std::endl;
      std::cout << std::setw(12) << file_name_ ;
      std::cout << std::setw(12) << std::hex << mem_start_;
      std::cout << std::setw(12) << std::hex << physical_mem_start_;
      std::cout << std::setw(12) << read_or_write_;
      std::cout << std::setw(12) << std::hex << file_length_;
      if( qType == "d" ) {
        std::cout << std::setw(12) << std::dec << cylinder_;
      }
      std::cout << std::endl;
      printPageTable();
      std::cout << "--";
    }
    std::cout << std::endl;
  }

  // When process has finished CPU burst, recalculate tau
  void recalculateTau( float alpha ) {
    tau_next_ = ( alpha * tau_previous_ ) + ( (1.0 - alpha) * t_previous_ );
    tau_next_remaining_ = tau_next_;
    number_of_bursts_++;
    resetTPreviousToZero();
  }

  void updatePageTable(int frame) { page_table_.push_back(frame); }
  std::vector<int> pageTable() { return page_table_; }

private:
  int pid_;
  unsigned int process_size_;
  unsigned int pages_needed_;
  // Used by devices
  std::string file_name_;
  int mem_start_;
  int physical_mem_start_;
  char read_or_write_;
  int file_length_;
  // Used for disk scheduling algorithm - C-LOOK
  int cylinder_;

  // Used for preemptive CPU scheduling - shortest job remaining algorithm
  float tau_next_;
  float tau_previous_;
  float t_previous_;
  float tau_next_remaining_;
  float cpu_usage_;
  int number_of_bursts_;

  std::vector<int> page_table_;

  void printPageTable() {
    std::cout << "--page table: ";
    for(int i = 0; i < page_table_.size(); ++i) {
      std::cout << std::hex << page_table_[i];
      if(i < page_table_.size()-1) {
        std::cout << ", ";
      }
    }
    std::cout << std::endl;
  }
};

struct compare {
  bool operator () ( PCB* lhs, PCB* rhs ) {
    // If same burst estimate remaining, use pid to compare
    if( lhs->burstEstimateRemaining() == rhs->burstEstimateRemaining() ) {
      return lhs->PID() > rhs->PID();
    }
    return lhs->burstEstimateRemaining() > rhs->burstEstimateRemaining();
  }
};

struct ComparePcbSize {
    bool operator() ( PCB* lhs, PCB* rhs) {
        return (lhs->processSize() > rhs->processSize());
    }
};

struct FrameInfo {
  int pid_;
  int page_;
  bool free_;

  inline void setFrameToFree() { free_ = true; }
  void updateFrameToNotFree(int pid, int page) {
    pid_ = pid;
    page_ = page;
    free_ = false;
  }

  FrameInfo( int pid = 0, int page = 0, bool free = false )
    : pid_( pid ), page_( page ), free_( free ) {}
};

class OSSimulator {
public:

  bool runOS() {
    int numPrinters = 0, numHDDs = 0, numFlashDrives = 0, systemTotalProcesses = 0;
    float historyParameter, initialBurstEstimate, systemTotalCpuTime = 0.0;
    unsigned int maxProcessSize, pageSize;
    unsigned long sizeOfMemory;

    // Map disk number to number of cylinders on that disk
    std::unordered_map<int, int> cylindersOnDisk;
    // Map disk number to current location of disk head for that disk
    std::unordered_map<int, int> locationsOfDiskHeads;
    // Map disk number to location of current max and min cyliders on disk
    std::unordered_map< int, std::pair<int, int> > diskMinAndMax;

    // Initialize OS with number of devices
    sysGen(numPrinters, numHDDs, numFlashDrives, historyParameter, initialBurstEstimate,
           cylindersOnDisk, sizeOfMemory, maxProcessSize, pageSize);

    //Initialize all disk heads to index 0, the first cylinder on each disk
    initializeDiskHeads( locationsOfDiskHeads, numHDDs );
    // Initialize all disk's current max and min cylinders to 0, number of cyliders
    initializeMinAndMax( diskMinAndMax, cylindersOnDisk, numHDDs );

    unsigned int numFrames = sizeOfMemory / pageSize;
    // Initialize the free frame list to contain all frames in memory
    // i.e. Size of memory/page size
    std::deque<int> freeFrameList;
    initializeFreeFrameList( freeFrameList, numFrames );
    // Initialize the frame table to contain only free frames
    std::vector<FrameInfo> frameTable(numFrames);
    initializeFrameTable( frameTable, numFrames );

    // Create a vector of each type of device containing the queues for each
    // individual device
    std::vector< std::deque<PCB*> > printers(numPrinters);
    std::vector< std::vector< std::deque<PCB*> > > hardDrives(numHDDs);
    std::vector< std::deque<PCB*> > flashDrives(numFlashDrives);

    // Initialize a Vector of sizeof(numDisks), each containing a vector of sizeof(numCylinders)
    // which each contain a deque of the processes on that cylinder
    initializeEachDisk( numHDDs, hardDrives, cylindersOnDisk );

    // CPU is represented by a PCB pointer, initialize first PID to 0
    PCB* CPU = NULL;
    next_pid_ = 0;
    // Ready Queue
    std::priority_queue<PCB*, std::vector<PCB*>, compare> readyQ;
    // Job pool
    std::vector<PCB*> jobPool;

    while(true) {
      std::string keyboard_input = getInput();
      if( checkInput(keyboard_input, numPrinters, numHDDs, numFlashDrives) ) {
        // Exit program
        if(keyboard_input == "exit") {
          break;
        }

          char input_request = keyboard_input[0];
          switch (input_request) {
            // INTERRUPT: System "Snapshot"
            case 'S':
            {
              std::string snapshot_input = snapshotInput();

              std::cout << std::endl;
              std::cout << "----System Average Burst Time: " << systemAverageCpuTime(systemTotalCpuTime, systemTotalProcesses) << std::endl << std::endl;

              switch( snapshot_input[0] ) {
                case 'c':
                  printCPU( CPU );
                  std::cout << std::endl;
                  break;
                case 'r':
                  printReadyQueue( readyQ );
                  std::cout << std::endl;
                  break;
                case 'p':
                  printDeviceQueues( printers, snapshot_input );
                  std::cout << std::endl;
                  break;
                case 'd':
                 printDiskQueues( snapshot_input, hardDrives, locationsOfDiskHeads, diskMinAndMax );
                  std::cout << std::endl;
                  break;
                case 'f':
                  printDeviceQueues( flashDrives, snapshot_input );
                  std::cout << std::endl;
                  break;
                case 'm':
                  printFrameTable( frameTable );
                  printFreeFrameList( freeFrameList );
                  break;
                case 'j':
                  printJobPool(jobPool);
                  break;
                case 'a':
                  printCPU( CPU );
                  std::cout << std::endl;
                  printReadyQueue( readyQ );
                  std::cout << std::endl;
                  printDeviceQueues( printers, "p" );
                  std::cout << std::endl;
                  printDiskQueues( "d", hardDrives, locationsOfDiskHeads, diskMinAndMax );
                  std::cout << std::endl;
                  printDeviceQueues( flashDrives, "f" );
                  std::cout << std::endl;
                  break;
              }
              break;
            }
            // INTERRUPT: Arrival of process
            case 'A':
            {
              unsigned int size = getProcessSize();
              // Create new process
              PCB* arriving_pcb = new PCB(get_next_pid_and_increment(), initialBurstEstimate, size);
              int pagesNeeded = size / pageSize + (size % pageSize != 0);
              arriving_pcb->updatePagesNeeded( pagesNeeded );

              // Reject process if larger than maximum process size
              if(arriving_pcb->processSize() > maxProcessSize) {
                std::cout << "The size of Process " << arriving_pcb->PID() << " exceeds the maximum process size.\n";
                std::cout << "The process has been rejected.\n";
                break;
              }
              else if( pagesNeeded > freeFrameList.size() ) {
                jobPool.push_back(arriving_pcb);
                std::sort(jobPool.begin(), jobPool.end(), ComparePcbSize());
              }
              else {
                loadProcessIntoMemory(arriving_pcb, freeFrameList, frameTable);
                // Add new process to ready queue
                readyQ.push(arriving_pcb);
                if( !cpu_empty(CPU) ) {
                  cpuProcessBookkeeping( CPU, historyParameter );
                }
                // If CPU is not occupied, add first process in Ready Queue to CPU
                if( cpu_empty(CPU) ) {
                  updateCpuProcessFromReadyQueue( CPU, readyQ );
                }
                compareCpuProcessToTopofReadyQ( CPU, readyQ );
              }
              break;
            }
            // SYSTEM CALL: Terminate process in CPU
            case 't':
            {
              if( !cpu_empty(CPU) ) {
                cpuProcessBookkeeping( CPU, historyParameter );
                PCB* old_process = CPU;
                // Free memory used by old process
                freeMemory(old_process, freeFrameList, frameTable);
                // Add processes waiting in job pool to memory if enough memory
                // has been freed by terminated process
                std::vector<PCB*>::iterator itr = jobPool.begin();
                while( itr != jobPool.end() && (*itr)->pagesNeeded() > freeFrameList.size() ) {
                  ++itr;
                }
                while( itr != jobPool.end() && (*itr)->pagesNeeded() <= freeFrameList.size() ) {
                  loadProcessIntoMemory( *itr, freeFrameList, frameTable);
                  readyQ.push(*itr);
                  itr = jobPool.erase(itr);
                }

                updateCpuProcessFromReadyQueue( CPU, readyQ );
                // Recycle memory of process that was terminated
                reportTerminatingProcessToAccountingModule( old_process );
                //update system-wide variables
                systemTotalProcesses++;
                systemTotalCpuTime += old_process->totalCpuTime();
                delete old_process;
              }
              else {
                std::cout << "No process in CPU, nothing to terminate.\n";
              }
              break;
            }
            // INTERRUPT: Find and kill specified process and free its memory
            case 'K':
            {
              int pid_to_kill = atoi(keyboard_input.substr(1).c_str());
              bool found = false;
              // Search Job Pool
              found = deleteFromJobPool(pid_to_kill, jobPool);
              if( !found ) {
                found = deleteFromCPU(pid_to_kill, CPU, readyQ, historyParameter,
                           systemTotalProcesses, systemTotalCpuTime, freeFrameList,
                           frameTable, jobPool);
              }
              if( !found ) {
                found = deleteFromReadyQ(pid_to_kill, readyQ, freeFrameList, frameTable,
                             jobPool, systemTotalProcesses, systemTotalCpuTime);
              }
              if( !found ) {
                found = deleteFromPrinterorFlashQ(pid_to_kill, printers, freeFrameList, frameTable,
                             jobPool, systemTotalProcesses, systemTotalCpuTime, readyQ);
              }
              if( !found ) {
                found = deleteFromHardDriveQ(pid_to_kill, hardDrives, freeFrameList, frameTable,
                            jobPool, systemTotalProcesses, systemTotalCpuTime, readyQ);
              }
              if( !found ) {
                found = deleteFromPrinterorFlashQ(pid_to_kill, flashDrives, freeFrameList,
                             frameTable, jobPool, systemTotalProcesses, systemTotalCpuTime, readyQ);
              }
              if( !found ) {
                std::cout << "Process not found in system. Nothing to kill.\n";
              }
              else {
                std::cout << "Process killed.\n";
              }
              break;
            }
            // INTERRUPT: Printer has completed task at the front of its queue
            // Pop process from specified device Q and push to ready Q
            case 'P':
            {
              if( !validDeviceNumber( keyboard_input, printers ) )
                break;
              if( !cpu_empty(CPU) ) {
                cpuProcessBookkeeping( CPU, historyParameter );
              }
              moveProcessToReadyQueue( keyboard_input, printers, readyQ );
              if( cpu_empty(CPU) ) {
                updateCpuProcessFromReadyQueue( CPU, readyQ );
              }
              compareCpuProcessToTopofReadyQ( CPU, readyQ );
              break;
            }
            // SYSTEM CALL: Process in CPU has requested a printer
            // Finished with CPU burst, ready for I/O burst
            // Move process from CPU to spefied device Q, move next process in ready Q to CPU
            case 'p':
            {
              if( !cpu_empty(CPU) ) {
                if( !validDeviceNumber( keyboard_input, printers ) )
                  break;
                // recalculate tau and update all other potential changes
                // MAY NEED TO CHANGE PROCESS BOOKKEEPING
                cpuProcessBookkeeping( CPU, historyParameter );
                CPU->recalculateTau( historyParameter );
                getProcessParametersAndUpdate(input_request, CPU, keyboard_input, cylindersOnDisk, pageSize);
                moveProcessToDeviceQueue( keyboard_input, CPU, printers );
                updateCpuProcessFromReadyQueue( CPU, readyQ );
              }
              else {
                std::cout << "No process in CPU, invalid system call.\n";
              }
              break;
            }
            // INTERRUPT: HDD has completed task at the front of its queue
            // Pop process from specified device Q and push to ready Q
            case 'D':
            {
              if( !validDiskNumber( keyboard_input, hardDrives ) )
                break;
              if( moveProcessToReadyQueueFromDiskQueue( keyboard_input, hardDrives,
                                cylindersOnDisk, locationsOfDiskHeads,
                                diskMinAndMax,readyQ ) ) {
                if( !cpu_empty(CPU) ) {
                  cpuProcessBookkeeping( CPU, historyParameter );
                }
                if( cpu_empty(CPU) ) {
                  updateCpuProcessFromReadyQueue( CPU, readyQ );
                }
                compareCpuProcessToTopofReadyQ( CPU, readyQ );
              }
              break;
            }
            // SYSTEM CALL: Process in CPU has requested a hard drive
            // Move process from CPU to spefied device Q, move next process in ready Q to CPU
            case 'd':
            {
              if( !cpu_empty(CPU) ) {
                if( !validDiskNumber( keyboard_input, hardDrives ) )
                  break;
                cpuProcessBookkeeping( CPU, historyParameter );
                CPU->recalculateTau( historyParameter );
                getProcessParametersAndUpdate(input_request, CPU, keyboard_input, cylindersOnDisk, pageSize);
                int diskNumber = getDiskNumber( keyboard_input );
                moveProcessToDiskQueue( diskNumber, CPU, hardDrives, diskMinAndMax );
                updateCpuProcessFromReadyQueue( CPU, readyQ );
              }
              else {
                std::cout << "No process in CPU, invalid system call.\n";
              }
              break;
            }
            // INTERRUPT: Flash drive has completed task at the front of its queue
            // Pop process from specified device Q and push to ready Q
            case 'F':
            {
              if( !validDeviceNumber( keyboard_input, flashDrives ) )
                break;
              if( !cpu_empty(CPU) ) {
                cpuProcessBookkeeping( CPU, historyParameter );
              }
              moveProcessToReadyQueue( keyboard_input, flashDrives, readyQ );
              if( cpu_empty(CPU) ) {
                updateCpuProcessFromReadyQueue( CPU, readyQ );
              }
              compareCpuProcessToTopofReadyQ( CPU, readyQ );
              break;
            }
            // SYSTEM CALL: Process in CPU has requested a flash drive
            // Move process from CPU to spefied device Q, move next process in ready Q to CPU
            case 'f':
            {
              if( !cpu_empty(CPU) ) {
                if( !validDeviceNumber( keyboard_input, flashDrives ) )
                  break;
                cpuProcessBookkeeping( CPU, historyParameter );
                CPU->recalculateTau( historyParameter );
                getProcessParametersAndUpdate(input_request, CPU, keyboard_input, cylindersOnDisk, pageSize);
                moveProcessToDeviceQueue( keyboard_input, CPU, flashDrives );
                updateCpuProcessFromReadyQueue( CPU, readyQ );
              }
              else {
                std::cout << "No process in CPU, invalid system call.\n";
              }
              break;
            }
          }
        }
      }
    return true;
  }

private:

  int next_pid_;

  inline int get_next_pid_and_increment() { return(next_pid_++); }

  bool cpu_empty( PCB* cpu ) {
    if( cpu == NULL ) {
      return true;
    }
    return false;
  }

  void updateCpuProcessFromReadyQueue( PCB*& cpu, std::priority_queue<PCB*, std::vector<PCB*>, compare>& ready ) {
    if( ready.empty() ) {
      cpu = NULL;     // Nothing to add, CPU is empty
    }
    else {
      cpu = ready.top();
      ready.pop();
    }
  }

  //Initialize system with number of devices of each type, history parameter and burst estimate
  bool sysGen( int& p, int& d, int& f, float& h, float& b,
               std::unordered_map<int, int>& cylindersOnDisk, unsigned long& sizeOfMemory,
               unsigned int& maxProcessSize, unsigned int& pageSize) {
    p = numDevices( "printers" );
    d = numDevices( "hard drives" );
    f = numDevices( "flash drives" );

    h = historyParameter();
    b = burstEstimate();
    mapCylindersToDisks(d, cylindersOnDisk);

    pageSize = getPageSize();
    sizeOfMemory = getMemorySize(pageSize);
    maxProcessSize = getMaxProcessSize(sizeOfMemory);

    return true;
  }

  void initializeFreeFrameList( std::deque<int>& freeFrames, unsigned int numFrames ) {
    for(unsigned int i = 0; i < numFrames; ++i ) {
      freeFrames.push_back(i);
    }
  }

  void initializeFrameTable( std::vector<FrameInfo>& frameTable, unsigned long numFrames ) {
    FrameInfo frame;
    frame.setFrameToFree();
    for(unsigned int i = 0; i < numFrames; ++i ) {
      frameTable[i] = frame;
    }
  }

  void printFrameTable( std::vector<FrameInfo> frameTable ) {
    std::cout << "----frame table\n";
    std::cout << std::left;
    std::cout << std::setw(8) << "Frame #";
    std::cout << std::setw(4) << "PID";
    std::cout << std::setw(7) << "Page #";
    std::cout << std::endl;

    for(int i = 0; i < frameTable.size(); ++i) {
      if(!frameTable[i].free_) {
        std::cout << std::setw(8) << i;
        std::cout << std::setw(4) << frameTable[i].pid_;
        std::cout << std::setw(7) << std::hex << frameTable[i].page_;
        std::cout << std::endl;
      }
    }
    std::cout << std::endl;
  }

  void printFreeFrameList( std::deque<int> freeFrameList ) {
    std::cout << "---free frame list\n";
    for(int i = 0; i < freeFrameList.size(); ++i) {
      std::cout << std::hex << freeFrameList[i];
      if(i != freeFrameList.size()-1) {
        std::cout <<  ", ";
      }
    }
    std::cout << std::endl << std::endl;
  }

  void printJobPool( std::vector<PCB*> jobPool ) {
    std::cout << "----job pool\n";
    std::cout << std::left;
    std::cout << std::setw(4) << "PID";
    std::cout << std::setw(5) << "Size";
    std::cout << std::endl;

    for(int i = 0; i < jobPool.size(); ++i) {
        std::cout << std::setw(4) << jobPool[i]->PID();
        std::cout << std::setw(5) << jobPool[i]->processSize();
        std::cout << std::endl;
    }
    std::cout << std::endl;
  }

  void loadProcessIntoMemory(PCB*& pcb, std::deque<int>& freeFrameList,
                             std::vector<FrameInfo>& frameTable) {
    int pages = pcb->pagesNeeded();
    for(int i = 0; i < pages; ++i) {
      int frame = freeFrameList.front();
      freeFrameList.pop_front();
      frameTable[frame].updateFrameToNotFree(pcb->PID(), i);
      pcb->updatePageTable(frame);
    }
  }

  void freeMemory( PCB*& pcb, std::deque<int>& freeFrameList, std::vector<FrameInfo>& frameTable) {
    for(int i = 0; i < pcb->pageTable().size(); ++i) {
      int frame = pcb->pageTable()[i];
      frameTable[frame].setFrameToFree();
      freeFrameList.push_back(frame);
      pcb->pageTable().clear();
    }
  }

  bool deleteFromJobPool( int pid, std::vector<PCB*>& jobPool ) {
    std::vector<PCB*>::iterator itr = jobPool.begin();
    for( ; itr != jobPool.end(); ++itr ) {
      if( (*itr)->PID() == pid ) {
        jobPool.erase(itr);
        return true;
      }
    }
    return false;
  }

  bool deleteFromCPU( int pid, PCB*& CPU, std::priority_queue<PCB*, std::vector<PCB*>, compare>& readyQ,
         float& historyParameter, int& systemTotalProcesses, float& systemTotalCpuTime,
         std::deque<int>& freeFrameList, std::vector<FrameInfo>& frameTable, std::vector<PCB*>& jobPool ) {
    if( !cpu_empty(CPU) && CPU->PID() == pid ) {
      cpuProcessBookkeeping( CPU, historyParameter );
      PCB* old_process = CPU;
      // Free memory used by old process
      freeMemory(old_process, freeFrameList, frameTable);
      // Add processes waiting in job pool to memory if enough memory
      // has been freed by terminated process
      std::vector<PCB*>::iterator itr = jobPool.begin();
      while( itr != jobPool.end() && (*itr)->pagesNeeded() > freeFrameList.size() ) {
        ++itr;
      }
      while( itr != jobPool.end() && (*itr)->pagesNeeded() <= freeFrameList.size() ) {
        loadProcessIntoMemory( *itr, freeFrameList, frameTable);
        readyQ.push(*itr);
        itr = jobPool.erase(itr);
      }

      updateCpuProcessFromReadyQueue( CPU, readyQ );
      // Recycle memory of process that was terminated
      reportTerminatingProcessToAccountingModule( old_process );
      //update system-wide variables
      systemTotalProcesses++;
      systemTotalCpuTime += old_process->totalCpuTime();
      delete old_process;

      return true;
    }
    return false;
  }

  bool deleteFromReadyQ( int pid, std::priority_queue<PCB*, std::vector<PCB*>, compare>& readyQ,
           std::deque<int>& freeFrameList, std::vector<FrameInfo>& frameTable,
           std::vector<PCB*>& jobPool, int& systemTotalProcesses, float& systemTotalCpuTime ) {
    std::vector<PCB*> copy;
    while( !readyQ.empty() && readyQ.top()->PID() != pid ) {
      copy.push_back(readyQ.top());
      readyQ.pop();
    }
    if( readyQ.top()->PID() == pid ) {
      PCB* to_kill = readyQ.top();
      readyQ.pop();
      for(int i = 0; i < copy.size(); ++i) {
        readyQ.push(copy[i]);
      }

      // Free memory used by old process
      freeMemory(to_kill, freeFrameList, frameTable);
      // Add processes waiting in job pool to memory if enough memory
      // has been freed by terminated process
      std::vector<PCB*>::iterator itr = jobPool.begin();
      while( itr != jobPool.end() && (*itr)->pagesNeeded() > freeFrameList.size() ) {
        ++itr;
      }
      while( itr != jobPool.end() && (*itr)->pagesNeeded() <= freeFrameList.size() ) {
        loadProcessIntoMemory( *itr, freeFrameList, frameTable);
        readyQ.push(*itr);
        itr = jobPool.erase(itr);
      }
      reportTerminatingProcessToAccountingModule( to_kill );
      //update system-wide variables
      systemTotalProcesses++;
      systemTotalCpuTime += to_kill->totalCpuTime();

      delete to_kill;
      return true;
    }
    else {
      return false;
    }
  }

  bool deleteFromPrinterorFlashQ( int pid, std::vector< std::deque<PCB*> >& device_type,
            std::deque<int>& freeFrameList, std::vector<FrameInfo>& frameTable,
            std::vector<PCB*>& jobPool, int& systemTotalProcesses, float& systemTotalCpuTime,
            std::priority_queue<PCB*, std::vector<PCB*>, compare>& readyQ ) {
    for(int i = 0; i < device_type.size(); ++i) {
      if(!device_type[i].empty()) {
        std::deque<PCB*>::iterator it = device_type[i].begin();
        while( it != device_type[i].end() && (*it)->PID() != pid ) {
          ++it;
        }
        PCB* to_kill = *it;
        if( to_kill->PID() == pid ) {
          // Free memory used by old process
          freeMemory(to_kill, freeFrameList, frameTable);
          // Add processes waiting in job pool to memory if enough memory
          // has been freed by terminated process
          std::vector<PCB*>::iterator itr = jobPool.begin();
          while( itr != jobPool.end() && (*itr)->pagesNeeded() > freeFrameList.size() ) {
            ++itr;
          }
          while( itr != jobPool.end() && (*itr)->pagesNeeded() <= freeFrameList.size() ) {
            loadProcessIntoMemory( *itr, freeFrameList, frameTable);
            readyQ.push(*itr);
            itr = jobPool.erase(itr);
          }
          reportTerminatingProcessToAccountingModule( to_kill );
          //update system-wide variables
          systemTotalProcesses++;
          systemTotalCpuTime += to_kill->totalCpuTime();

          device_type[i].erase(it);
          delete to_kill;
          return true;
        }
      }
    }
    return false;
  }

  bool deleteFromHardDriveQ( int pid, std::vector< std::vector< std::deque<PCB*> > >& hardDrives,
                std::deque<int>& freeFrameList, std::vector<FrameInfo>& frameTable,
                std::vector<PCB*>& jobPool, int& systemTotalProcesses, float& systemTotalCpuTime,
                std::priority_queue<PCB*, std::vector<PCB*>, compare>& readyQ  ) {
    for(int i = 0; i < hardDrives.size(); ++i) {
      for(int j = 0; j < hardDrives[i].size(); ++j) {
        if( !hardDrives[i][j].empty() ) {


          std::deque<PCB*>::iterator it = hardDrives[i][j].begin();
          while( it != hardDrives[i][j].end() && (*it)->PID() != pid ) {
            ++it;
          }
          PCB* to_kill = *it;
          if( to_kill->PID() == pid ) {
            // Free memory used by old process
            freeMemory(to_kill, freeFrameList, frameTable);
            // Add processes waiting in job pool to memory if enough memory
            // has been freed by terminated process
            std::vector<PCB*>::iterator itr = jobPool.begin();
            while( itr != jobPool.end() && (*itr)->pagesNeeded() > freeFrameList.size() ) {
              ++itr;
            }
            while( itr != jobPool.end() && (*itr)->pagesNeeded() <= freeFrameList.size() ) {
              loadProcessIntoMemory( *itr, freeFrameList, frameTable);
              readyQ.push(*itr);
              itr = jobPool.erase(itr);
            }
            reportTerminatingProcessToAccountingModule( to_kill );
            //update system-wide variables
            systemTotalProcesses++;
            systemTotalCpuTime += to_kill->totalCpuTime();

            hardDrives[i][j].erase(it);
            delete to_kill;
            return true;
          }
      }
      }
    }
    return false;
  }

  void initializeDiskHeads( std::unordered_map<int, int>& locationsOfDiskHeads, int numDisks ) {
    for(int i = 0; i < numDisks; ++i ) {
      locationsOfDiskHeads[i] = 0;
    }
  }

  void initializeMinAndMax( std::unordered_map< int, std::pair<int, int> >& diskMinAndMax,
                            std::unordered_map<int, int>& cylindersOnDisk, int numDisks ) {
    for(int i = 0; i < numDisks; ++i ) {
      std::pair<int, int> minAndMax;
      minAndMax.first = 0;
      minAndMax.second = cylindersOnDisk[i];
      diskMinAndMax[i] = minAndMax;
    }
  }

  // Get number of devices for SysGen section from user
  int numDevices( std::string device ) {
    int numDevices;
    std::cout << "Enter the number of " << device << " for this system, ";
    std::cout << "and press the Enter key.\n";
    std::cin >> numDevices;

      //Number of devices can't be negative or user entered a non-integer string
      while( numDevices < 0 || std::cin.fail() ) {
        std::cout << "Number of devices must be a positive integer. ";
        std::cout << "Please try again:\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> numDevices;
      }
    return numDevices;
  }

  void mapCylindersToDisks( int numDisks, std::unordered_map<int, int>& cylindersOnDisk ) {
    int numCylinders;
    for(int i = 0; i < numDisks; ++i) {
      numCylinders = getNumCylinders(i);
      cylindersOnDisk[i] = numCylinders;
    }
  }

  int getNumCylinders( int currentDisk ) {
    int numCylinders;
    std::cout << "Enter the number of cylinders on disk " << currentDisk;
    std::cout << " and press the Enter key.\n";
    std::cin >> numCylinders;

      //Number of devices can't be negative or user entered a non-integer string
      while( numCylinders < 1 || std::cin.fail() ) {
        std::cout << "Number of cylinders must be a positive integer greater than zero. ";
        std::cout << "Please try again:\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> numCylinders;
      }
    return numCylinders;
  }

  // Get the history paramater - alpha
  // The history parameter must be 0 >= alpha >= 1
  float historyParameter() {
    float historyParam;
    std::cout << "Enter the history parameter for this system, a number between 0 and 1, ";
    std::cout << "and press the Enter key.\n";
    std::cin >> historyParam;

      //Number of devices can't be negative or user entered a non-integer string
      while( historyParam < 0 || historyParam > 1 || std::cin.fail() ) {
        std::cout << "History parameter must be a positive number between 0 and 1, inclusive. ";
        std::cout << "Please try again:\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> historyParam;
      }
    return historyParam;
  }

  // Get the initial burst estimate, in milliseconds
  int burstEstimate() {
    int intitialBurstEst;
    std::cout << "Enter the initial burst estimate for this system, ";
    std::cout << "and press the Enter key.\n";
    std::cin >> intitialBurstEst;

      //Number of devices can't be negative or user entered a non-integer string
      while( intitialBurstEst < 0 || std::cin.fail() ) {
        std::cout << "Initial burst estimate must be a positive integer. ";
        std::cout << "Please try again:\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> intitialBurstEst;
      }
    return intitialBurstEst;
  }

  unsigned int getPageSize() {
    unsigned int pageSize;
    std::cout << "Enter the size of a page for this system (in words) ";
    std::cout << "and press the Enter key.\n";
    std::cin >> pageSize;

      //Number of devices can't be negative or user entered a non-integer string
      while( !isPowerOfTwo(pageSize) || std::cin.fail() ) {
        std::cout << "Page size must be a positive integer and a power of two. ";
        std::cout << "Please try again:\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> pageSize;
      }
    return pageSize;
  }

  bool isPowerOfTwo(unsigned int x) {
    return (x != 0) && ((x & (x - 1)) == 0);
  }

  unsigned long getMemorySize(unsigned int pageSize) {
    unsigned long sizeOfMemory;
    std::cout << "Enter the total size of memory for this system (in words) ";
    std::cout << "and press the Enter key.\n";
    std::cin >> sizeOfMemory;

      //Number of devices can't be negative or user entered a non-integer string
      while( !memoryIsDivisibleByPageSize(pageSize, sizeOfMemory) || std::cin.fail() ) {
        std::cout << "Size of memory must be a positive integer and a multiple of the system page size. ";
        std::cout << "Please try again:\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> sizeOfMemory;
      }
    return sizeOfMemory;
  }

  bool memoryIsDivisibleByPageSize(unsigned int pageSize, unsigned long memSize) {
    return memSize % pageSize == 0;
  }

  unsigned int getMaxProcessSize(unsigned long sizeOfMemory) {
    unsigned int maxProcessSize;
    std::cout << "Enter the maximum size of a process for this system (in words) ";
    std::cout << "and press the Enter key.\n";
    std::cin >> maxProcessSize;

      //Number of devices can't be negative or user entered a non-integer string
      while( maxProcessSize > sizeOfMemory || std::cin.fail() ) {
        std::cout << "Maximum size of a process must be a positive integer and be smaller than the total size of system memory. ";
        std::cout << "Please try again:\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> maxProcessSize;
      }
    return maxProcessSize;
  }

  unsigned int getProcessSize() {
    unsigned int size;
    std::cout << "Enter the size of the arriving process (in words) ";
    std::cout << "and press the Enter key.\n";
    std::cin >> size;

      //Number of devices can't be negative or user entered a non-integer string
      while( std::cin.fail() ) {
        std::cout << "Maximum size of a process must be a positive integer. ";
        std::cout << "Please try again:\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> size;
      }
    return size;
  }

  std::string getInput() {
    std::string command;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Enter an interrupt or system call (or exit to end program)";
    std::cout << " and press Enter key: " << std::endl;
    std::cin >> command;
    return command;
  }

  // Keyboard input must be one of the following:
  // A - Arrival of Process interrupt
  // S - Snapshot interrupt
  // t - Terminate Process system call
  // Device (P - printer, D - HDD, F- flash drive) + number of device interrupt
  // Device (p - print, d - HDD, f - flash drive) + number of device system Call
  bool checkInput(std::string input_string, int p, int d, int f) {
    if(std::regex_match(input_string, std::regex("A|S|t|(K|P|D|F|p|d|f)(\\d+)|exit"))) {
      char first = input_string[0];
      if((input_string.length() > 1) && (input_string != "exit") && (first != 'K')) {
          if(!goodDeviceNumber(input_string, p, d, f)) {
            return false;
          }
        }
      return true;
    }
    else {
      std::cout << "Not a valid Interrupt or System Call. Please try again.\n";
      return false;
    }
  }

  void initializeEachDisk( int num_disks, std::vector< std::vector< std::deque< PCB* > > >& disks,
                           std::unordered_map<int, int>& cylinders ) {
    for( int i = 0; i < num_disks; ++i ) {
      int num_cylinders = cylinders[i];
      disks[i].resize( num_cylinders );
    }
  }

  // Get amount of CPU time used by process currently in the CPU
  float getAmountOfCpuTimeUsed() {
    float cpuTime;
    std::cout << "Enter the CPU time used by the current process ";
    std::cout << "and press the Enter key.\n";
    std::cin >> cpuTime;

      //Number of devices can't be negative or user entered a non-integer string
      while( cpuTime < 0 || std::cin.fail() ) {
        std::cout << "CPU time used must be a positive number. ";
        std::cout << "Please try again:\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> cpuTime;
      }
    return cpuTime;
  }

  void cpuProcessBookkeeping( PCB* CPU, float historyParameter ) {
    // Query user for amount of CPU time used by process currently in CPU
    float amountOfCpuTimeUsed = getAmountOfCpuTimeUsed();
    // Update tau_next_remaining for PCB currently in CPU
    CPU->updateBurstEstimateRemaining( amountOfCpuTimeUsed );
    // Add the current CPU time to t_previous
    CPU->updateTPrevious( amountOfCpuTimeUsed );
    // Add the current CPU time to cpu_usage
    CPU->updateCPUUsage( amountOfCpuTimeUsed );
  }

  void compareCpuProcessToTopofReadyQ( PCB*& CPU, std::priority_queue<PCB*, std::vector<PCB*>, compare>& readyQ ) {
    if( !readyQ.empty() && !cpu_empty(CPU) ) {
      if( readyQ.top()->burstEstimateRemaining() < CPU->burstEstimateRemaining() ) {
        PCB* old_process = CPU;
        updateCpuProcessFromReadyQueue( CPU, readyQ );
        readyQ.push( old_process );
      }
    }
  }

  // Check that the device number entered by the user is less than the number
  // of devices of that type
  bool goodDeviceNumber(std::string input_string, int p, int d, int f) {
    // User counts from 1, program counts from 0, so subtract 1
    int device_number = atoi(input_string.substr(1).c_str()) - 1;
    char device = input_string[0];

    switch(device) {
      case 'P':
      case 'p':
        if(device_number > p) {
          std::cout << "Not a valid device number. Please try again.\n";
          return false;
        }
        break;
      case 'D':
      case 'd':
        if(device_number > d) {
          std::cout << "Not a valid device number. Please try again.\n";
          return false;
        }
        break;
      case 'F':
      case 'f':
        if(device_number > f) {
          std::cout << "Not a valid device number. Please try again.\n";
          return false;
        }
        break;
      default:
        std::cout << "Not a valid device. Please try again.\n";
        return false;
    }
    return true;
  }

  int getDiskNumber( std::string input_string ) {
    return atoi(input_string.substr(1).c_str());
  }

  // Input must be r, p, d, f, a
  // r for ready, p for printers, d for HDDs, f for flash drives, or a for all
  std::string snapshotInput() {
    std::string snapshot_type;
    std::cout << "Enter snapshot type (r-ready, p-printers, d-hdds, f-flash, m-memory, j-jobs, a-all):\n";
    std::cin >> snapshot_type;
    // Input must be r, p, d, f, a
    while( !(std::regex_match(snapshot_type, std::regex( "r|p|d|f|a|m|j" )))  ) {
      std::cout << "Not a valid snapshot type.\n";
      std::cout << "Enter r for ready, p for printers, d for HDDs, f for flash drives, m for memory, j for job pool, or a for all):\n";
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cin >> snapshot_type;
    }
    return snapshot_type;
  }

  // Print all values of all processes in all queues of a given device type
  void printDeviceQueues( std::vector< std::deque<PCB*> > device_type, std::string device_code ) {
    for(int i = 0; i < device_type.size(); ++i) {
      std::cout << "----" + device_code + std::to_string(i) << std::endl;
      printHeaders(device_code);
      printDeviceHeaders(device_code);
      if(!device_type[i].empty()) {
        for(std::deque<PCB*>::iterator it = device_type[i].begin(); it != device_type[i].end(); ++it) {
          PCB* pcb = *it;
          pcb->printPcbContents(device_code);
        }
      }
    }
  }

  void printDiskQueues( std::string device_code, std::vector< std::vector< std::deque< PCB* > > > disks,
                       std::unordered_map<int, int>& locationsOfDiskHeads,
                       std::unordered_map<int, std::pair<int, int> > minAndMax ) {
   for(int i = 0; i < disks.size(); ++i) {
     std::cout << "----" + device_code + std::to_string(i) << std::endl;
     printHeaders(device_code);
     printDeviceHeaders(device_code);

     int disk_head = locationsOfDiskHeads[i];
     int original_disk_head = disk_head;
     int min = minAndMax[i].first;
     int max = minAndMax[i].second;
     do {
       if(!disks[i][disk_head].empty()) {
         for(std::deque<PCB*>::iterator it = disks[i][disk_head].begin(); it != disks[i][disk_head].end(); ++it) {
           PCB* pcb = *it;
           pcb->printPcbContents(device_code);
         }
       }
       if(++disk_head >= max) {
         disk_head = min;
       }
     } while(disk_head != original_disk_head);
   }
 }

  // Print PID of current process in CPU
  void printCPU( PCB* CPU ) {
    std::cout << "----CPU\n";
    printHeaders("c");
    if( cpu_empty(CPU) ) {
      std::cout << "No process in CPU.\n";
    }
    else {
      CPU->printPcbContents("c");
    }
  }

  // Print PID's of all processes in the Ready Queue
  void printReadyQueue( std::priority_queue<PCB*, std::vector<PCB*>, compare> readyQ ) {
    std::cout << "----ready queue\n";
    printHeaders("r");
    while( !readyQ.empty() ) {
      readyQ.top()->printPcbContents("r");
      readyQ.pop();
    }
  }

  void printHeaders( std::string qType ) {
      std::cout << std::left;
      std::cout << std::setw(12) << "PID";
      std::cout << std::setw(12) << "CPU_Time";
      std::cout << std::setw(12) << "Avg_Burst";
      std::cout << std::setw(12) << "Tau_Next";
      std::cout << std::setw(12) << "Tau_Nxt_Rem";
      std::cout << std::endl;
  }

  void printDeviceHeaders( std::string qType ) {
    std::cout << std::setw(12) << "Filename";
    std::cout << std::setw(12) << "Log_Addr";
    std::cout << std::setw(12) << "Phys_Addr";
    std::cout << std::setw(12) << "R/W";
    std::cout << std::setw(12) << "File_Len";
    if( qType == "d" ) {
      std::cout << std::setw(12) << "Cylinder";
    }
    std::cout << std::endl;
  }

  // User enters file name, where file starts in memory, read or write, and
  // length of file and updates process information. Called when a process
  // is added to a device queue.
  void getProcessParametersAndUpdate(char device, PCB* CPU, std::string input,
                 std::unordered_map<int, int> cylindersOnDisk, unsigned int pageSize) {
    std::string file_name, rw;
    int mem_start, file_len, cylinder;

    std::cout << "Enter file name and press Enter key: \n";
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::getline( std::cin, file_name );
    CPU->updateFileName( file_name );

    std::cout << "Enter starting location in memory and press Enter key: \n";
    std::cin >> std::hex >> mem_start;
    while( mem_start < 0 || std::cin.fail() ) {
      std::cout << "Starting location in memory must be a positive hexidecimal integer.\n";
      std::cout << "Please enter a hexidecimal integer:\n";
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cin >> std::hex >> mem_start;
    }
    int physical_mem_start = getPhysicalAddress( mem_start, pageSize, CPU);
    while( physical_mem_start < 0 ) {
      std::cout << "Page Fault, address out of range.\n";
      std::cout << "Enter starting location in memory and press Enter key: \n";
      std::cin >> std::hex >> mem_start;
      while( mem_start < 0 || std::cin.fail() ) {
        std::cout << "Starting location in memory must be a positive hexidecimal integer.\n";
        std::cout << "Please enter a hexidecimal integer:\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> std::hex >> mem_start;
      }
      physical_mem_start = getPhysicalAddress( mem_start, pageSize, CPU);
    }
    CPU->updateMemStart( mem_start );
    CPU->updatePhysicalMemStart( physical_mem_start );
    std::cout << "The physical address is " << std::hex << physical_mem_start << ".\n";

    // Printers only write
    if( device == 'p' ) {
      rw = "w";
    }
    else {
      std::cout << "Enter r or w for Read or Write to device and press Enter key: \n";
      std::cin >> rw;
      while( rw != "r" && rw != "w" ) {
        std::cout << "Input not valid, enter r or w and press Enter: \n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> rw;
      }
    }
    CPU->updateReadOrWrite( rw[0] );

    // Only need file length on a write
    if( rw == "w") {
      std::cout << "Enter file length and press Enter key: \n";
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cin >> std::hex >> file_len;
      while( mem_start < 0 || std::cin.fail() ) {
        std::cout << "File length must be a positive hexidecimal integer.\n";
        std::cout << "Please enter a hexidecimal integer:\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> std::hex >> file_len;
      }
      CPU->updateFileLength( file_len );
    }
    if( device == 'd' ) {
      int diskNumber = atoi( input.substr(1).c_str() );
      std::cout << "Enter the cylinder to access: \n";
      std::cin >> std::dec >> cylinder;

      while( cylinder >= cylindersOnDisk[diskNumber] ) {
        std::cout << "Cylinder to access greater than number of cylinders on disk. ";
        std::cout << "Please enter a cylinder to access greater than 0 and less than " << cylindersOnDisk[diskNumber] << std::endl;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> std::dec >> cylinder;
      }
      CPU->updateCylinder( cylinder );
    }
  }

  int getPhysicalAddress(int logical, int pageSize, PCB* pcb) {
    int page = logical / pageSize;
    int offset = logical % pageSize;
    // page fault, address out of range
    if( page >= pcb->pageTable().size() || page < 0 ) {
      return -1;
    }
    else {
      int frame = pcb->pageTable()[page];
      int physical = (frame * pageSize) + offset;
      return physical;
    }
  }

  bool validDeviceNumber( std::string input, std::vector< std::deque<PCB*> >& device_vec ) {
      int device_queue_number = atoi( input.substr(1).c_str() );
      if( device_queue_number > device_vec.size() -1 ) {
        std::cout << "Not a valid device number. Please try again.\n";
        return false;
      }
      return true;
  }

  bool validDiskNumber( std::string input, std::vector< std::vector< std::deque<PCB*> > >& device_vec ) {
      int device_queue_number = atoi( input.substr(1).c_str() );
      if( device_queue_number > device_vec.size() -1 ) {
        std::cout << "Not a valid device number. Please try again.\n";
        return false;
      }
      return true;
  }

  void moveProcessToDeviceQueue( std::string input, PCB*& CPU, std::vector< std::deque<PCB*> >& device_vec ) {
    if( CPU != NULL ) {
      int device_queue_number = atoi( input.substr(1).c_str() );
      PCB* current_process = CPU;
      // Add process to device's queue
      device_vec[ device_queue_number ].push_back( current_process );
    }
  }

  void moveProcessToDiskQueue( int diskNumber, PCB* CPU,
                               std::vector< std::vector< std::deque< PCB*> > >& hardDrives,
                               std::unordered_map< int, std::pair<int, int> >& diskMinAndMax ) {
    if( CPU != NULL ) {
      PCB* current_process = CPU;
      // Add process to device's queue
      int cylinder = CPU->cylinder();
      hardDrives[ diskNumber ][ cylinder ].push_back( CPU );
      if( CPU->cylinder() < diskMinAndMax[ diskNumber ].first ) {
        diskMinAndMax[ diskNumber ].first = CPU->cylinder();
      }
      if( CPU->cylinder() > diskMinAndMax[ diskNumber ].second ) {
        diskMinAndMax[ diskNumber ].second = CPU->cylinder();
      }
    }
  }

  void moveProcessToReadyQueue( std::string input, std::vector< std::deque<PCB*> >& device_vec,
                                std::priority_queue<PCB*, std::vector<PCB*>, compare>& ready ) {
    int device_queue_number = atoi( input.substr(1).c_str() );
    if( !(device_vec[device_queue_number].empty()) ) {
      PCB* current_process = device_vec[ device_queue_number ].front();
      device_vec[ device_queue_number ].pop_front();
      current_process->resetPcbValues();
      ready.push( current_process );
    }
  }

  bool moveProcessToReadyQueueFromDiskQueue( std::string input,
                                std::vector< std::vector< std::deque<PCB*> > >& disk_vec,
                                std::unordered_map<int, int> cylindersOnDisk,
                                std::unordered_map<int, int>& locationsOfDiskHeads,
                                std::unordered_map<int, std::pair<int, int> > minAndMax,
                                std::priority_queue<PCB*, std::vector<PCB*>, compare>& ready ) {
    int disk_number = atoi( input.substr(1).c_str() );
    //Need to check if there is anything in the disk
    int disk_head = locationsOfDiskHeads[ disk_number ];
    int original_disk_head = disk_head;
    int cylinders = cylindersOnDisk[ disk_number ];
    int min = minAndMax[ disk_number ].first;
    int max = minAndMax[ disk_number ].second;
    bool disk_contains_processes = true;

    int i = 1;
    while ( disk_vec[ disk_number ][ disk_head ].empty() ) {
      // reached end of disk
      if( i >= cylinders ) {
        disk_contains_processes = false;
        return disk_contains_processes;
      }
      ++i;

      if( disk_head > max ) {
        disk_head = min;
      }
      else {
        ++disk_head;
      }
    }

    PCB* next_to_service = disk_vec[ disk_number ][ disk_head ].front();
    disk_vec[ disk_number ][ disk_head ].pop_front();
    next_to_service->resetPcbValues();
    locationsOfDiskHeads[ disk_number ] = disk_head;
    ready.push( next_to_service );
    return disk_contains_processes;
  }

  void reportTerminatingProcessToAccountingModule( PCB* process ) {
    std::cout << "Terminating Process PID: " << process->PID() << std::endl;
    std::cout << "Terminating Process Total CPU Time: " << process->totalCpuTime() << std::endl;
    std::cout << "Terminating Process Average Burst Time: " << process->averageBurstTime() << std::endl;
  }

  float systemAverageCpuTime( float systemTotalCpuTime, int systemTotalProcesses ) {
    return (systemTotalProcesses > 0 ? systemTotalCpuTime / systemTotalProcesses : 0);
  }

};

int main() {
  OSSimulator OS;
  OS.runOS();

  return 0;
}
