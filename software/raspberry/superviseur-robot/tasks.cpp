/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_TBATTERY 20
#define PRIORITY_TCOMROBOT 26

#define PERIODE_BATTERIE 500000000
#define PERIODE_MOVE     100000000
#define PERIODE_WD        30000000

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_codeErr, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_checkcomrobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_servInit, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_newMove, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_checkcomrobot, "th_checkcomrobot", 0, PRIORITY_TCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_watchdog, "th_watchdog", 0, PRIORITY_TCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::CheckBattery, this)) {
            cerr << "Error task start: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_checkcomrobot, (void(*)(void*)) & Tasks::CheckComRobot, this)) {
            cerr << "Error task start: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_watchdog, (void(*)(void*)) & Tasks::RefreshWD, this)) {
            cerr << "Error task start: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
    }
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    rt_sem_v(&sem_servInit);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    while(1) {
        rt_sem_p(&sem_servInit, TM_INFINITE);
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        status = monitor.Open(SERVER_PORT);
        rt_mutex_release(&mutex_monitor);

        cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

        if (status < 0) throw std::runtime_error {
            "Unable to start server on port " + std::to_string(SERVER_PORT)
        };
        monitor.AcceptClient(); // Wait the monitor client
        cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
        rt_sem_broadcast(&sem_serverOk);
    }
}

/**
 * @brief Thread reseting the watchdog counter
 */
void Tasks::RefreshWD(void *arg) {
    int err;
    Message * msgRcv;
    cout << "Reset Watchdog " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task RefreshWD starts here                                                        */
    /*******************************************************u*******************************///
    rt_task_set_periodic(NULL, TM_NOW, PERIODE_WD);
    while(1){
        rt_task_wait_period(NULL);
        if (ROBOT_WD) {
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgRcv = robot.Write(robot.ReloadWD()) ;
            rt_mutex_release(&mutex_robot);
            cout << "RefreshWD: robot.Write(robot.ReloadWD()) = " << msgRcv->ToString() << endl << flush ;
            CheckCom(msgRcv);
        }
    }
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */

void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) { //Sur perte de connexion avec le moniteur: consigne de stop envoyée au robot suivi de la fermeture des communications
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = false;
            rt_mutex_release(&mutex_robotStarted);
            robot.Write(robot.Stop());
            robot.Close() ; 
            monitor.Close() ; 
            rt_sem_v(&sem_servInit); //Réactivation du sémaphore d'initialisation pour l'écoute sur les ports serveurs
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_CLOSE)) { //idem sans moniteur 
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = false;
            rt_mutex_release(&mutex_robotStarted);
            robot.Write(robot.Stop()); 
            robot.Close() ; 
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            ROBOT_WD = true ;
            rt_sem_v(&sem_startRobot); 
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_RESET)) {
            robot.Close(); 
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
            rt_sem_v(&sem_newMove);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}  
/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;
        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    Message * msgToSend;
    Message * msgSend;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_startRobot, TM_INFINITE);
       
        //Message * msgToSend = (ROBOT_WD) ? robot.StartWithWD() : robot.StartWithoutWD();
        cout << "ROBOT_WD = " << ROBOT_WD << endl ;
        if(ROBOT_WD){
            
            msgToSend = robot.StartWithWD();
            cout << "Robot to start with WD" << endl ;
        } else {
            cout << "Robot to start without WD" << endl ;
            msgToSend = robot.StartWithoutWD();
        }
        cout << " msg sent : (" << msgToSend->GetID() << ")" << endl;
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(msgToSend);
        rt_mutex_release(&mutex_robot);
        // Check robot communication with a counter
        int err = 1 ; 
        if (msgSend->CompareID(MESSAGE_ANSWER_COM_ERROR))    
                err = -1;
        rt_mutex_acquire(&mutex_codeErr, TM_INFINITE);
        err_com_robot = err ; 
        rt_mutex_release(&mutex_codeErr);
        rt_sem_v(&sem_checkcomrobot);
        

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = true;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    bool rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, PERIODE_MOVE);

    while (1) {
        Message *msgSend;
        rt_task_wait_period(NULL);
        rt_sem_p(&sem_newMove, TM_INFINITE);
        //cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove << endl << flush;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
            // Check robot communication with a counter
            int err = 1 ; 
            if (msgSend->CompareID(MESSAGE_ANSWER_COM_ERROR))
                err = -1 ;
            rt_mutex_acquire(&mutex_codeErr, TM_INFINITE);
            err_com_robot = err ; 
            rt_mutex_release(&mutex_codeErr);
            rt_sem_v(&sem_checkcomrobot);
        }
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

void Tasks::CheckBattery(void *arg) {
    Message *batLvl = 0;
    int codeErr = 0;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, PERIODE_BATTERIE);
    while (1) {
        rt_task_wait_period(NULL);
        //cout << "acquisition mutex_robotStarted" << endl << flush; 
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        bool rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        //cout << "released mutex_robotStarted" << endl << flush;
        //cout << "Battery: "; 
        //cout << "acquisition mutex_robot" << endl << flush;
        
        if(rs){
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            batLvl = robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));
            rt_mutex_release(&mutex_robot);
            // Check robot communication with a counter
            if (batLvl->CompareID(MESSAGE_ANSWER_COM_ERROR)) {
                codeErr = -1;
            } else {
                codeErr = 1;
            }
            rt_mutex_acquire(&mutex_codeErr, TM_INFINITE);
            err_com_robot = codeErr;
            rt_mutex_release(&mutex_codeErr);
            rt_sem_v(&sem_checkcomrobot);
            
            if(0 == batLvl) {
                //cout << "erreur de communication avec le robot" << endl << flush; 
            }else{
                //cout << batLvl << endl << flush;
                //cout << "Write batLvl" << endl << flush;
                //RCLS
                WriteInQueue(&q_messageToMon, batLvl);
                if(batLvl->CompareID((MessageID)BATTERY_EMPTY)){
                    cout << "battery EMPTY!!!!" << endl << flush;
                    exit(EXIT_FAILURE);
                }
                //cout << "Written batLvl" << endl << flush;
            }
            
        }else{
            //cout << "Robot not started !!!!!" << endl << flush;
        }

        
    }
}

/**
 * @brief Check for robot communication using a counter
 */
void Tasks::CheckComRobot(void *arg) {
    int counter = 0;
    int codeErr;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_checkcomrobot, TM_INFINITE);
        rt_mutex_acquire(&mutex_codeErr, TM_INFINITE);
        codeErr = err_com_robot;
        rt_mutex_release(&mutex_codeErr);
        //code err = -1 si err -> incrementation du counter; err = 1 si ok -> decrementation jusqu'à 0
        if((counter -= codeErr) < 0){
            counter = 0 ; 
        }
        cout << "Com Robot Counter =  " << counter << endl << flush;
        // Check counter
        if (counter >= 3) {
            // Deconnecter robot com
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Close();
            rt_mutex_release(&mutex_robot);
            
            cout << "Communication failure (counter >= 3)" << endl << flush;
            
            // Send message to monitor
            WriteInQueue(&q_messageToMon, new Message(MESSAGE_ROBOT_COM_CLOSE));                
        }
        /*rt_mutex_acquire(&mutex_codeErr, TM_INFINITE);
        err_com_robot = 0;
        rt_mutex_release(&mutex_codeErr);*/
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

void Tasks::CheckCom(Message * msgRcv){
    int err;
    if (msgRcv->CompareID(MESSAGE_ANSWER_ACK)){
        err = 1 ;
    } else {
        err = -1 ; 
    }
    rt_mutex_acquire(&mutex_codeErr, TM_INFINITE);
    err_com_robot = err ;
    rt_mutex_release(&mutex_codeErr);
    rt_sem_v(&sem_checkcomrobot);
}
