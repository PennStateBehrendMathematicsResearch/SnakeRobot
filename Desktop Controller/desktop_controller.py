#!/usr/bin/env python
# -*- coding: utf-8 -*-
import bluetooth
import tkinter
import tkinter.scrolledtext
import tkinter.messagebox
import threading
import time
from collections import deque
import pygame.event
import pygame.joystick
import enum
import socket

import re

ROBOT_DEVICE_NAME = 'SnakeRobot'
ROBOT_SERVICE_NAME = b'Dev B'

class BluetoothThread(threading.Thread):
    class ConnectedState(enum.Enum):
        DISCONNECTED = 0
        WAITING_FOR_CONNECTION = 1
        CONNECTED = 2

    ERR_RESOURCE_TEMPORARILY_UNAVAILABLE = 11
    ERR_CONNECTION_RESET_BY_PEER = 104
    WINDOWS_BLUETOOTH_WOULD_BLOCK_KEYWORDS = 'non-blocking socket operation could not be completed immediately'
    def __init__(self, responseDelimChar, connStateChangedHandler, responseHandler, deviceName, serviceName):
        self.responseDelimChar = responseDelimChar
        self.connStateChangedHandler = connStateChangedHandler
        self.responseHandler = responseHandler
        
        self.disconnectFlagLock = threading.Lock()
        self.disconnectFlag = False

        self.connectedStateLock = threading.Lock()
        self.connectedState = BluetoothThread.ConnectedState.DISCONNECTED

        self.deviceName = deviceName
        self.serviceName = serviceName
        
        self.sendDataQueue = deque()
        
        # self.receivedCallback = dataReceivedCallback
        threading.Thread.__init__(self)
    
    @staticmethod    
    def getBluetoothErrorCode(ex):
        # This method is a hack, but it seems to be best way at this time of extracting an error code from a bluetooth.btcommon.BluetoothError, as it does not
        # seem to provide any variables.
        compiledExpression = re.compile('\((\d+).*\)')
        expressionMatches = compiledExpression.match(str(ex))
        
        if expressionMatches != None:
            return int(expressionMatches.group(1))
        else:
            return None
        
    def enqueueForSending(self, data):
        # self.sendDataLock.acquire()
        self.sendDataQueue.append(data)
        # self.sendDataLock.release()
    
    def sendDisconnectSignal(self):
        self.disconnectFlagLock.acquire()
        self.disconnectFlag = True
        self.disconnectFlagLock.release()
        
        # if(self.getConnectedState() is BluetoothThread.ConnectedState.WAITING_FOR_CONNECTION):
        #     print('Forcing socket closed...')
        #     self.server_sock.
    
    def getDisconnectSignal(self):
        self.disconnectFlagLock.acquire()
        returnValue = self.disconnectFlag
        self.disconnectFlagLock.release()
        return returnValue

    def getConnectedState(self):
        self.connectedStateLock.acquire()
        returnValue = self.connectedState
        self.connectedStateLock.release()
        return returnValue

    def _endConnection(self, client_sock):
        self.disconnectFlagLock.acquire()
        self.disconnectFlag = False
        self.disconnectFlagLock.release()

        self.connectedStateLock.acquire()
        self.connectedState = BluetoothThread.ConnectedState.DISCONNECTED
        self.connectedStateLock.release()
                
        client_sock.close()
                
        self.connStateChangedHandler(False)
        
    def run(self):
        client_sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
        
        print('Discovering robot address and port...')

        self.connectedStateLock.acquire()
        self.connectedState = BluetoothThread.ConnectedState.WAITING_FOR_CONNECTION
        self.connectedStateLock.release()

        discoverableDevices = bluetooth.discover_devices(lookup_names=True)
        deviceMAC = None
        devicePort = None
        connectionSucceeded = False

        for thisDevice in discoverableDevices:
            if thisDevice[1] == self.deviceName:
                deviceMAC = thisDevice[0]
                print('Found robot Bluetooth radio; MAC:', deviceMAC)
                break

        if deviceMAC and not(self.getDisconnectSignal()):
            availableServices = bluetooth.find_service(uuid=bluetooth.SERIAL_PORT_CLASS)
            
            for thisService in availableServices:
                if thisService['host'] == deviceMAC and (thisService['name'] == self.serviceName or thisService['name'] == self.serviceName.decode('utf-8')):
                    devicePort = thisService['port']
                    print('Found robot Bluetooth service; port:', devicePort)
                    break

            if devicePort and not(self.getDisconnectSignal()):
                print('Connecting...')
                client_sock.connect((deviceMAC, devicePort))
                connectionSucceeded = True

        if connectionSucceeded and not(self.getDisconnectSignal()):
            self.connectedStateLock.acquire()
            self.connectedState = BluetoothThread.ConnectedState.CONNECTED
            self.connectedStateLock.release()
            
            self.connStateChangedHandler(True)
            
            print('Socket accepted')
            
            # client_sock.send("Connection started\n")
            client_sock.setblocking(False)
        
            receivingBuffer = ''
            # peerDisconnected = False
            connectionLost = False
            try:
                while not(self.getDisconnectSignal() or connectionLost):
                    # self.sendDataLock.acquire()
                    
                    while self.sendDataQueue:
                        sendData = self.sendDataQueue.popleft()
                        client_sock.send(sendData)
                        # client_sock.send(self.sendDataQueue.popleft())
                        print('Sent data:', sendData)

                    startTime = time.perf_counter()
                    receivedData = None
                    try:
                        receivedData = client_sock.recv(4096)

                        if not(receivedData):
                            connectionLost = True
                            print('The connection has been lost.')
                    except bluetooth.btcommon.BluetoothError as ex:
                        if BluetoothThread.getBluetoothErrorCode(ex) != BluetoothThread.ERR_RESOURCE_TEMPORARILY_UNAVAILABLE:
                            raise
                    except OSError as ex: # Windows platforms
                        if BluetoothThread.WINDOWS_BLUETOOTH_WOULD_BLOCK_KEYWORDS not in str(ex):
                            raise
                        else:
                            elapsedTime = time.perf_counter() - startTime
                            if elapsedTime >= 0.01:
                                print('recv call duration:', elapsedTime)

                    if receivedData:
                        receivingBuffer += receivedData.decode('ascii')
                        
                        print('Received data:', receivedData.decode('ascii'))
                        # print('Bytes:', [thisByte for thisByte in receivedData])
                        # self.receivedCallback(receivedData)
                        
                        responseStrings = receivingBuffer.split(self.responseDelimChar)
                        
                        if len(responseStrings) >= 2:
                            for index in range(0, len(responseStrings) - 1):
                                self.responseHandler(responseStrings[index])
                            
                            receivingBuffer = responseStrings[len(responseStrings) - 1]
                    
                    if not(connectionLost):
                        time.sleep(0.02)
            except bluetooth.btcommon.BluetoothError as ex:
                if BluetoothThread.getBluetoothErrorCode(ex) != BluetoothThread.ERR_CONNECTION_RESET_BY_PEER:
                    raise
            finally:
                self._endConnection(client_sock)
        else:
           self._endConnection(client_sock) 
                    
        
def robotResponseHandler(responseStr):
    print('Received ', responseStr)

    if not(windowClosing):
        robotResponseText.config(state=tkinter.NORMAL)
        robotResponseText.insert(tkinter.END, responseStr + '\n')
        robotResponseText.config(state=tkinter.DISABLED)
        robotResponseText.see('end')

def robotConnStateChangedHandler(connected):
    newText = ('Disconnect' if connected else 'Connect')
    # print(newText)
    print('Connection state:', 'Connected' if connected else 'Disconnected')
    
    if not(connected):
        stopHoldCommand()

    if not(windowClosing):
        connectionStateButton.config(text=newText, state=tkinter.NORMAL)
        customCommandButton.config(state=(tkinter.NORMAL if connected else tkinter.DISABLED))
        holdToggleButton.config(state=(tkinter.NORMAL if connected else tkinter.DISABLED))

        if not(connected):
            holdToggleButton.config(text='Hold')

def toggleRobotConnection():
    global applicationBluetoothThread
    if not(applicationBluetoothThread) or not(applicationBluetoothThread.is_alive()):
        connectionStateButton.config(text='Connecting...', state=tkinter.DISABLED)
        applicationBluetoothThread = BluetoothThread('\n', robotConnStateChangedHandler, robotResponseHandler, ROBOT_DEVICE_NAME, ROBOT_SERVICE_NAME)
        applicationBluetoothThread.start()
    else:
        connectionStateButton.config(text='Disconnecting...', state=tkinter.DISABLED)
        applicationBluetoothThread.sendDisconnectSignal()

def sendCustomCommand():
    applicationBluetoothThread.enqueueForSending(customCommandInput.get() + '\n')

def setJoystick(joystickID, joystickName):
    global currentJoystick

    joystickSelectValue.set(joystickName)

    if currentJoystick != None:
        currentJoystick.quit()
    
    currentJoystick = pygame.joystick.Joystick(joystickID)
    currentJoystick.init()

def refreshJoysticks():
    global currentJoystick
    joystickSelect['menu'].delete(0, tkinter.END)
    
    if currentJoystick != None:
        currentJoystick.quit()
        currentJoystick = None
        joystickSelectValue.set('')
    
    pygame.quit()
    pygame.init()
        
    for thisID in range(0, pygame.joystick.get_count()):
        joystickObj = pygame.joystick.Joystick(thisID)
        joystickName = joystickObj.get_name()
        joystickSelect['menu'].add_command(label=joystickName, command=(lambda: setJoystick(thisID, joystickName)))

def updateJoystickInput():
    if joystickEnableCheckboxState.get() and currentJoystick and applicationBluetoothThread and (applicationBluetoothThread.getConnectedState() is BluetoothThread.ConnectedState.CONNECTED):
        pygame.event.pump()
        # print(currentJoystick.get_name(), 'axes:', currentJoystick.get_axis(0), currentJoystick.get_axis(1))

        try:
            joystickFrequency = -(float(joystickFreqScaleEntry.get())) * currentJoystick.get_axis(1)
            joystickTurnOffset = float(joystickTurnAngleScaleEntry.get()) * currentJoystick.get_axis(0)
            
            applicationBluetoothThread.enqueueForSending('runimm %.3f %.3f\n' % (joystickFrequency, joystickTurnOffset))
        except ValueError:
            print('Invalid value in joystick parameters.')

    mainWindow.after(250, updateJoystickInput)

def sendHoldCommand(holdFrequency, holdTurnAngle):
    global holdJobID
##    if(applicationBluetoothThread and applicationBluetoothThread.getConnectedState() is BluetoothThread.ConnectedState.CONNECTED)
    applicationBluetoothThread.enqueueForSending('runimm %.3f %.3f\n' % (holdFrequency, holdTurnAngle))

    holdJobIDLock.acquire()
    holdJobID = mainWindow.after(250, sendHoldCommand, holdFrequency, holdTurnAngle)
    holdJobIDLock.release()
##    else:
##        holdJobIDLock.acquire()
##        holdJobID = None
##        holdJobIDLock.release()

def stopHoldCommand():
    global holdJobID

    if not(windowClosing): 
        holdJobIDLock.acquire()
        
        if holdJobID != None:
            mainWindow.after_cancel(holdJobID)
            holdJobID = None
        
        holdJobIDLock.release()

    if(applicationBluetoothThread and applicationBluetoothThread.getConnectedState() is BluetoothThread.ConnectedState.CONNECTED):
        applicationBluetoothThread.enqueueForSending('runimm 0.0 0.0\n')

def toggleHold():
    global holdJobID

    holdJobIDLock.acquire()
    
    if holdJobID == None:
        holdJobIDLock.release()

        try:
            sendHoldCommand(float(holdFrequencyEntry.get()), float(holdTurnEntry.get()))
            holdToggleButton.config(text='Stop')
        except ValueError:
            print('Invalid value in hold parameters.')
    else:
        holdJobIDLock.release()
        stopHoldCommand()
        holdToggleButton.config(text='Hold')

windowClosing = False
def onWindowClose():
    global windowClosing
    if applicationBluetoothThread and (applicationBluetoothThread.getConnectedState() is not BluetoothThread.ConnectedState.DISCONNECTED):
        if tkinter.messagebox.askyesno('Quit Controller', 'Terminate open connection and quit?'):
            windowClosing = True
            connectionStateButton.config(text='Disconnecting...', state=tkinter.DISABLED)
            applicationBluetoothThread.sendDisconnectSignal()

            print('Waiting for Bluetooth thread to end...')
            applicationBluetoothThread.join()

            mainWindow.destroy()
    else:
        windowClosing = True
        mainWindow.destroy()

def joystickCommandEnabledChanged():
    if joystickEnableCheckboxState.get():
        joystickFreqScaleEntry.config(state=tkinter.DISABLED)
        joystickTurnAngleScaleEntry.config(state=tkinter.DISABLED)
    else:
        joystickFreqScaleEntry.config(state=tkinter.NORMAL)
        joystickTurnAngleScaleEntry.config(state=tkinter.NORMAL)

# pygame.joystick.init()        

mainWindow = tkinter.Tk()
mainWindow.title('Bluetooth Snake Robot Controller')
connectionStateButton = tkinter.Button(mainWindow, text='Connect', command=toggleRobotConnection)

customCommandLabel = tkinter.Label(mainWindow, text='Custom command:')

customCommandInput = tkinter.Entry(mainWindow)

customCommandButton = tkinter.Button(mainWindow, text='Send', command=sendCustomCommand, state=tkinter.DISABLED)

robotResponseTextLabel = tkinter.Label(mainWindow, text='Received responses:')
robotResponseText = tkinter.scrolledtext.ScrolledText(mainWindow, wrap=tkinter.WORD, state=tkinter.DISABLED)

joystickControlFrame = tkinter.LabelFrame(mainWindow, text='Joystick', padx=5, pady=5)

joystickEnableCheckboxState = tkinter.IntVar(joystickControlFrame, value=0)
joystickEnableCheckbox = tkinter.Checkbutton(joystickControlFrame, text="Enable joystick control", variable=joystickEnableCheckboxState, command=joystickCommandEnabledChanged)

joystickSelectValue = tkinter.StringVar(joystickControlFrame)
joystickSelect = tkinter.OptionMenu(joystickControlFrame, joystickSelectValue, '[Joysticks not loaded]')

joystickRefreshButton = tkinter.Button(joystickControlFrame, text='Refresh', command=refreshJoysticks)

joystickFreqScaleLabel = tkinter.Label(joystickControlFrame, text='Joystick frequency scale:')
joystickFreqScaleEntry = tkinter.Entry(joystickControlFrame)
joystickFreqScaleEntry.insert(tkinter.END, '0.5')

joystickTurnAngleScaleLabel = tkinter.Label(joystickControlFrame, text='Joystick turn angle scale:')
joystickTurnAngleScaleEntry = tkinter.Entry(joystickControlFrame, text='7.0')
joystickTurnAngleScaleEntry.insert(tkinter.END, '7.0')

holdFrequencyEntryLabel = tkinter.Label(mainWindow, text='Frequency:')
holdFrequencyEntry = tkinter.Entry(mainWindow)

holdTurnEntryLabel = tkinter.Label(mainWindow, text='Turn offset:')
holdTurnEntry = tkinter.Entry(mainWindow)

holdToggleButton = tkinter.Button(mainWindow, text='Hold', command=toggleHold, state=tkinter.DISABLED)
holdJobIDLock = threading.Lock()
holdJobID = None

connectionStateButton.grid(row=0, column=0, columnspan=5)
joystickControlFrame.grid(row=1, column=0, columnspan=5, sticky=tkinter.EW, padx=5)

joystickEnableCheckbox.grid(row=0, column=0)
joystickSelect.grid(row=0, column=1, sticky=tkinter.EW)
joystickRefreshButton.grid(row=0, column=2, sticky=tkinter.W, padx=5)
joystickFreqScaleLabel.grid(row=1, column=0, sticky=tkinter.E)
joystickFreqScaleEntry.grid(row=1, column=1, sticky=tkinter.EW)
joystickTurnAngleScaleLabel.grid(row=1, column=2, sticky=tkinter.E)
joystickTurnAngleScaleEntry.grid(row=1, column=3, sticky=tkinter.EW)

customCommandLabel.grid(row=3, column=0, sticky=tkinter.E)
customCommandInput.grid(row=3, column=1, sticky=tkinter.EW)
customCommandButton.grid(row=3, column=2, sticky=tkinter.W, padx=(5, 0))
holdFrequencyEntryLabel.grid(row=4, column=0, sticky=tkinter.E)
holdFrequencyEntry.grid(row=4, column=1, sticky=tkinter.EW)
holdTurnEntryLabel.grid(row=4, column=2, sticky=tkinter.E)
holdTurnEntry.grid(row=4, column=3, sticky=tkinter.EW)
holdToggleButton.grid(row=4, column=4, sticky=tkinter.W, padx=(5, 0))
robotResponseTextLabel.grid(row=5, column=0, columnspan=5, sticky=tkinter.W)
robotResponseText.grid(row=6, column=0, columnspan=5, sticky=tkinter.NSEW)

mainWindow.rowconfigure(robotResponseText.grid_info()['row'], weight=1)

for thisIndex in range(mainWindow.grid_size()[0]):
    mainWindow.columnconfigure(thisIndex, weight=1)

for thisIndex in range(joystickControlFrame.grid_size()[0]):
    joystickControlFrame.columnconfigure(thisIndex, weight=1)

applicationBluetoothThread = None
# applicationBluetoothThread.enqueueForSending('testCommand\n')

currentJoystick = None

mainWindow.protocol("WM_DELETE_WINDOW", onWindowClose)
mainWindow.wait_visibility()
refreshJoysticks()
updateJoystickInput()
mainWindow.mainloop()
