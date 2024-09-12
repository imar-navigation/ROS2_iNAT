from math import *

class wgs84:
    '''wgs84 constants'''

    def __init__(self):
        pass

    def a(self):
        return 6378137.0

    def f(self):
        return 1.0 / 298.257223563

    def e2(self):
        return 2.0 * wgs84.f(self) - wgs84.f(self)**2

def rad2deg(ang):
    return [val * 180 / pi for val in ang]

def deg2rad(ang):
    return [val * pi / 180 for val in ang]

def normalizeToRange(value, startValue, endValue):
    width       = endValue - startValue
    offsetValue = value - startValue
    return ( offsetValue - ( floor( offsetValue / width ) * width ) ) + startValue


def C_ecef2ned(xLonLatAlt):
    sinLon = sin(xLonLatAlt[0])
    cosLon = cos(xLonLatAlt[0])
    sinLat = sin(xLonLatAlt[1])
    cosLat = cos(xLonLatAlt[1])
    return [[-sinLat * cosLon, -sinLat * sinLon, cosLat], [-sinLon, cosLon, 0], [-cosLat * cosLon, -cosLat * sinLon, -sinLat]]

def C_ned2ecef(xLonLatAlt):
    sinLon = sin(xLonLatAlt[0])
    cosLon = cos(xLonLatAlt[0])
    sinLat = sin(xLonLatAlt[1])
    cosLat = cos(xLonLatAlt[1])
    return [[-sinLat * cosLon, -sinLon, -cosLat * cosLon], [-sinLat * sinLon, cosLon, -cosLat * sinLon], [cosLat, 0, -sinLat]]

# RPY in Rad
# def EulerAng2RotMat(RollPitchYaw):
#     CosPhi0 = cos(RollPitchYaw[0])
#     CosPhi1 = cos(RollPitchYaw[1])
#     CosPhi2 = cos(RollPitchYaw[2])
#     SinPhi0 = sin(RollPitchYaw[0])
#     SinPhi1 = sin(RollPitchYaw[1])
#     SinPhi2 = sin(RollPitchYaw[2])
#     C11 = CosPhi1 * CosPhi2
#     C21 = SinPhi0 * SinPhi1 * CosPhi2 - CosPhi0 * SinPhi2
#     C31 = CosPhi0 * SinPhi1 * CosPhi2 + SinPhi0 * SinPhi2
#     C12 = CosPhi1 * SinPhi2
#     C22 = SinPhi0 * SinPhi1 * SinPhi2 + CosPhi0 * CosPhi2
#     C32 = CosPhi0 * SinPhi1 * SinPhi2 - SinPhi0 * CosPhi2
#     C13 = -SinPhi1
#     C23 = SinPhi0 * CosPhi1
#     C33 = CosPhi0 * CosPhi1  
    
#     return  [[C11, C12, C13], [C21, C22, C23], [C31, C32, C33]]

def RPY_To_RotMatNav2Body(RollPitchYaw):
    sroll = sin(RollPitchYaw[0])
    croll = cos(RollPitchYaw[0])
    spitch = sin(RollPitchYaw[1])
    cpitch = cos(RollPitchYaw[1])
    syaw = sin(RollPitchYaw[2])
    cyaw = cos(RollPitchYaw[2])
    C11 = cpitch * cyaw
    C21 = sroll * spitch * cyaw - croll * syaw
    C31 = croll * spitch * cyaw + sroll * syaw
    C12 = cpitch * syaw
    C22 = sroll * spitch * syaw + croll * cyaw
    C32 = croll * spitch * syaw - sroll * cyaw
    C13 = -spitch
    C23 = sroll * cpitch
    C33 = croll * cpitch  
    return  [[C11, C12, C13], [C21, C22, C23], [C31, C32, C33]]

def matVecAdd(vec1,vec2):
    resVec0=vec1[0] + vec2[0]
    resVec1=vec1[1] + vec2[1]
    resVec2=vec1[2] + vec2[2]
    return [resVec0, resVec1, resVec2]

def matVecMult(mat, vec):
    resVec0 = mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2]
    resVec1 = mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2]
    resVec2 = mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2]
    return [resVec0, resVec1, resVec2]

def matMatMult(matL, matR):
    resMat00 = matL[0][0] * matR[0][0] + matL[0][1] * matR[1][0] + matL[0][2] * matR[2][0]
    resMat10 = matL[1][0] * matR[0][0] + matL[1][1] * matR[1][0] + matL[1][2] * matR[2][0]
    resMat20 = matL[2][0] * matR[0][0] + matL[2][1] * matR[1][0] + matL[2][2] * matR[2][0]
    resMat01 = matL[0][0] * matR[0][1] + matL[0][1] * matR[1][1] + matL[0][2] * matR[2][1]
    resMat11 = matL[1][0] * matR[0][1] + matL[1][1] * matR[1][1] + matL[1][2] * matR[2][1]
    resMat21 = matL[2][0] * matR[0][1] + matL[2][1] * matR[1][1] + matL[2][2] * matR[2][1]
    resMat02 = matL[0][0] * matR[0][2] + matL[0][1] * matR[1][2] + matL[0][2] * matR[2][2]
    resMat12 = matL[1][0] * matR[0][2] + matL[1][1] * matR[1][2] + matL[1][2] * matR[2][2]
    resMat22 = matL[2][0] * matR[0][2] + matL[2][1] * matR[1][2] + matL[2][2] * matR[2][2]
    return [[resMat00, resMat01, resMat02], [resMat10, resMat11, resMat12], [resMat20, resMat21, resMat22]]

def matTranspose(R):
    return [[R[0][0], R[1][0], R[2][0]], [R[0][1], R[1][1], R[2][1]], [R[0][2], R[1][2], R[2][2]]]

def rotMat_To_quat(R):
    sqrtArg = 1 + R[0][0] + R[1][1] + R[2][2]
    if sqrtArg > 0:
        a = 1 / 2 * sqrt(sqrtArg)
    else:
        a = 0
    sqrtArg = 1 + R[0][0] - R[1][1] - R[2][2]
    if sqrtArg > 0:
        b = 1 / 2 * sqrt(sqrtArg)
    else:
        b = 0
    sqrtArg = 1 - R[0][0] + R[1][1] - R[2][2]
    if sqrtArg > 0:
        c = 1 / 2 * sqrt(sqrtArg)
    else:
        c = 0
    sqrtArg = 1 - R[0][0] - R[1][1] + R[2][2]
    if sqrtArg > 0:
        d = 1 / 2 * sqrt(sqrtArg)
    else:
        d = 0
    if max([a, b, c, d]) == a:
        b = 1 / 4 / a * (R[2][1] - R[1][2])
        c = 1 / 4 / a * (R[0][2] - R[2][0])
        d = 1 / 4 / a * (R[1][0] - R[0][1])
    elif max([a, b, c, d]) == b:
        a = 1 / 4 / b * (R[2][1] - R[1][2])
        c = 1 / 4 / b * (R[1][0] + R[0][1])
        d = 1 / 4 / b * (R[0][2] + R[2][0])
    elif max([a, b, c, d]) == c:
        a = 1 / 4 / c * (R[0][2] - R[2][0])
        b = 1 / 4 / c * (R[1][0] + R[0][1])
        d = 1 / 4 / c * (R[2][1] + R[1][2])
    else:
        a = 1 / 4 / d * (R[1][0] - R[0][1])
        b = 1 / 4 / d * (R[0][2] + R[2][0])
        c = 1 / 4 / d * (R[2][1] + R[1][2])
    return [a, b, c, d]

class iQuat:
    '''qauternion object class'''

    def __init__(self, initList):
        if len(initList) == 4:
            self.q = initList
        else:
            self.q = [1, 0, 0, 0]

    def toRpy(self):
        a = self.q[0]
        b = self.q[1]
        c = self.q[2]
        d = self.q[3]
        roll = atan2(2.0 * (c * d - a * b), 1.0 - 2.0 * b**2 - 2.0 * c**2)
        asinArg = -2.0 * (b * d + a * c)
        if asinArg > 1.0:
            asinArg = 1.0
        elif asinArg < -1.0:
            asinArg = -1.0
        pitch = asin(asinArg)
        yaw = atan2(2.0 * (b * c - a * d), 1.0 - 2.0 * c**2 - 2.0 * d**2)
        return [roll, pitch, yaw]

    def fromRpy(self, rpy):
        self.q[0] = cos(rpy[0] / 2.0) * cos(rpy[1] / 2.0) * cos(rpy[2] / 2.0) + sin(rpy[0] / 2.0) * sin(rpy[1] / 2.0) * sin(rpy[2] / 2.0)
        self.q[1] = -(sin(rpy[0] / 2.0) * cos(rpy[1] / 2.0) * cos(rpy[2] / 2.0) - cos(rpy[0] / 2.0) * sin(rpy[1] / 2.0) * sin(rpy[2] / 2.0))
        self.q[2] = -(cos(rpy[0] / 2.0) * sin(rpy[1] / 2.0) * cos(rpy[2] / 2.0) + sin(rpy[0] / 2.0) * cos(rpy[1] / 2.0) * sin(rpy[2] / 2.0))
        self.q[3] = -(cos(rpy[0] / 2.0) * cos(rpy[1] / 2.0) * sin(rpy[2] / 2.0) - sin(rpy[0] / 2.0) * sin(rpy[1] / 2.0) * cos(rpy[2] / 2.0))
        return self.q

    def toRotMat(self):
        a = self.q[0]
        b = self.q[1]
        c = self.q[2]
        d = self.q[3]
        R00 = a**2 + b**2 - c**2 - d**2
        R11 = a**2 - b**2 + c**2 - d**2
        R22 = a**2 - b**2 - c**2 + d**2
        R01 = 2.0 * (b * c - a * d)
        R10 = 2.0 * (b * c + a * d)
        R02 = 2.0 * (b * d + a * c)
        R20 = 2.0 * (b * d - a * c)
        R12 = 2.0 * (c * d - a * b)
        R21 = 2.0 * (c * d + a * b)
        return [[R00, R01, R02], [R10, R11, R12], [R20, R21, R22]]

    def fromRotMat(self, R):
        self.q = rotMat_To_quat(R)
        return self.q

    def transpose(self):
        if abs(self.q[0]) > 0.5:
            return [-self.q[0], self.q[1], self.q[2], self.q[3]]
        else:
            return [self.q[0], -self.q[1], -self.q[2], -self.q[3]]

class iRotMat:
    '''rotation matrix object class'''

    def __init__(self, initRotMat = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]):
        if len(initRotMat) == 3 and len(initRotMat[0]) == 3 and len(initRotMat[1]) == 3 and len(initRotMat[2]) == 3:
            self.R = initRotMat
    def data(self):
        return self.R

    def toQuat(self):
        return rotMat_To_quat(self.R)

    def toRpy(self):
        return [atan2(self.R[1][2], self.R[2][2]), asin(-self.R[0][2]), atan2(self.R[0][1], self.R[0][0])]

    def fromRpy(self, rpy):
        sroll = sin(rpy[0]);
        croll = cos(rpy[0]);
        spitch = sin(rpy[1]);
        cpitch = cos(rpy[1]);
        syaw = sin(rpy[2]);
        cyaw = cos(rpy[2]);
        self.R[0] = [cpitch * cyaw, cpitch * syaw, -spitch]
        self.R[1] = [sroll * spitch * cyaw - croll * syaw, sroll * spitch * syaw + croll * cyaw, sroll * cpitch]
        self.R[2] = [croll * spitch * cyaw + sroll * syaw, croll * spitch * syaw - sroll * cyaw, croll * cpitch]
        return self

    def transpose(self):
        self.R = matTranspose(self.R)
        return self

class iPosLonLatAlt:
    '''lon lat alt position object class'''

    def __init__(self, initPos = [0, 0, 0]):
        if len(initPos) == 3:
            if(initPos[0]<-180 or initPos[0]> 180):
                self.xECEF = initPos
                self.lonLatAlt = [0,0,0]
            else:    
                self.lonLatAlt = initPos
                self.xECEF= [0,0,0]


    def fromPosEcef(self, xECEF):
        a = wgs84.a(self)
        e2 = wgs84.e2(self)
        z = xECEF[2]
        z2 = z**2
        r2 = xECEF[0]**2 + xECEF[1]**2
        r  = sqrt(r2)
        a2 = a**2
        b2 = a**2 * (1.0 - e2)
        F = 54.0 * b2 * z**2
        G = r2 + (1.0 - e2) * z2 - e2 * (a2 - b2)
        c = e2**2 * F * r2 / G**3
        s = (1.0 + c + sqrt(c**2 + 2.0 * c))**(1.0 / 3.0);
        P = F / (3.0 * (s + 1.0 / s + 1.0)**2 * G**2)
        Q = sqrt(1.0 + 2.0 * e2**2 * P)
        r0 = -P * e2 * r / (1.0 + Q) + sqrt(abs(a2 / 2.0 * (1.0 + 1.0 / Q) - P * (1.0 - e2) * z2 / (Q * (1.0 + Q)) - P * r2 / 2.0))
        U = sqrt((r - e2 * r0)**2 + z2)
        V = sqrt((r - e2 * r0)**2 + (1.0 - e2) * z2)
        z0 = b2 * z / (a * V)
        self.lonLatAlt = [atan2(xECEF[1], xECEF[0]), atan2((z + (a2 - b2) / b2 * z0) , r), U * (1.0 - b2 / (a * V))]
        return self.lonLatAlt

    def toPosEcef(self,lonLatAlt):
        self.lonLatAlt = [lonLatAlt[0],lonLatAlt[1],lonLatAlt[2]];
        sinLon = sin(lonLatAlt[0]);
        cosLon = cos(lonLatAlt[0]);
        sinLat = sin(lonLatAlt[1]);
        cosLat = cos(lonLatAlt[1]);
        N = 6378137 / sqrt(1 - 6.69438002290079e-003 * sinLat**2);
        self.xECEF[0] = (N + lonLatAlt[2]) * cosLat * cosLon;
        self.xECEF[1] = (N + lonLatAlt[2]) * cosLat * sinLon;
        self.xECEF[2] = (N * 993.305619977099e-003 + lonLatAlt[2]) * sinLat
        return self.xECEF

