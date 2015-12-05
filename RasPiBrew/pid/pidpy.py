import time

class pidpy:
    def __init__(self, sampleTime, Kp, Ki, Kd, setDirectionReverse=False):
        print("sampleTime =%f",sampleTime)
        self.isAutomatic = True
        self.output = 0
        self.input = 0
        self.error = 0
        self.lastDInput = 0
        self.lastInput = 0
        self.ITerm = 0
        self.setOutputLimits(0, 100)
        self.sampleTime = sampleTime*1000
        self.isDirectionReverse = False
        self.setControllerDirection(setDirectionReverse)
        self.setTunings(Kp,Ki,Kd)
        self.lastTime = self.millis() - self.sampleTime
        self.k_lpf = 0.1*Kd
        self.lpf1 = (2.0 * self.k_lpf - self.sampleTime*1000) / (2.0 * self.k_lpf + self.sampleTime*1000)
        self.lpf2 = self.sampleTime*1000 / (2.0 * self.k_lpf + self.sampleTime*1000)
        self.lpf = 0

    def millis(self):
        return time.time()*1000
#        self.timer = self.timer+1
#        return self.timer

    def isTimeToRecalculate(self):
        #timeChange = self.millis()-self.lastTime
        #return timeChange>=self.sampleTime
        return True

    def compute(self, input, setPoint):
        if not self.isAutomatic:
            return False
        self.input = input
        self.setPoint = setPoint
        if self.isTimeToRecalculate():
            self.error = self.setPoint - input

            # prevent windup to full output of ITerm if we are far away from the setpoint
            if input<self.setPoint-100/self.kp:
                self.ITerm = self.outMin
                dInput = 0
            else:
                self.ITerm = self.ITerm + (self.ki*self.error)
                dInput = input - self.lastInput


            if self.ITerm > self.outMax:
                self.ITerm = self.outMax
            elif self.ITerm < self.outMin:
                self.ITerm = self.outMin

            self.lpf = self.lpf1*self.lpf + self.lpf2*(dInput + self.lastDInput)
            PTerm = self.kp*self.error
            #if PTerm<0:
            #    PTerm = 0

            print("p(%f) %f i(%f) %f d(%f) %f"%(self.kp,self.kp*self.error, self.ki,self.ITerm, self.kd, self.kd*self.lpf))
#            self.output = PTerm + self.ITerm - self.kd*dInput
            self.output = PTerm + self.ITerm - self.kd*self.lpf

            if self.output > self.outMax:
                self.output = self.outMax
            elif self.output < self.outMin:
                self.output = self.outMin

            self.lastInput = input
            self.lastDInput = dInput

            return True
        else:
            return False

    def calcPID_reg4(self, xk, tset, enable):
        if self.isAutomatic and not enable:
            setAutomatic = false
            self.setMode(setAutomatic)
            return false
        if not self.isAutomatic and enable:
            setAutomatic = true
            self.setMode(setAutomatic)

        self.compute(xk, tset)
        return self.output

    def setTunings(self, Kp, Ki, Kd):
        print("setTunings(%f,%f,%f)"%(Kp,Ki,Kd))
        if Kp<0 or Ki<0 or Kd<0:
            return

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        SampleTimeInSec = self.sampleTime/1000.0

        self.kp = Kp
        self.ki = (Kp/Ki) * SampleTimeInSec
        self.kd = Kp*Kd / SampleTimeInSec

        if self.isDirectionReverse:
            self.kp = 0 - self.kp
            self.ki = 0 - self.ki
            self.kd = 0 - self.kd

    def setSampleTime(self, newSampleTime):
        if newSampleTime<0:
            return

        ratio = newSampleTime/self.sampleTime
        self.ki = self.ki * ratio
        self.kd = self.kd / ratio
        self.sampleTime = newSampleTime

    def setOutputLimits(self, minOut, maxOut):
        if minOut>maxOut:
            return
        self.outMax = maxOut
        self.outMin = minOut

        if self.isAutomatic:
            if self.output >= self.outMax:
                self.output = self.outMax
            elif self.output <= self.outMin:
                self.output = self.outMin

    def setMode(self, setAutomatic):
        if setAutomatic:
            self.initialize()
        self.isAutomatic = setAutomatic

    def initialize(self):
        self.ITerm = self.output
        self.lastInput = self.input
        self.setOutputLimits(self.outMin, self.outMax)

    def setControllerDirection(self, setDirectionReverse):
        if self.isAutomatic and self.isDirectionReverse!=setDirectionReverse:
            self.kp = 0 - self.kp
            self.ki = 0 - self.ki
            self.kd = 0 - self.kd
