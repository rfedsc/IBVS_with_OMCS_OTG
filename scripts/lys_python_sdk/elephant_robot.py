import numpy as np

def CvtRotationMatrixToEulerAngle(pdtRotationMatrix):
  pdtEulerAngle = np.zeros(3)
  pdtEulerAngle[2] = np.arctan2(pdtRotationMatrix[1, 0], pdtRotationMatrix[0, 0])
  fCosRoll = np.cos(pdtEulerAngle[2])
  fSinRoll = np.sin(pdtEulerAngle[2])
  pdtEulerAngle[1] = np.arctan2(-pdtRotationMatrix[2, 0], (fCosRoll * pdtRotationMatrix[0, 0]) + (fSinRoll * pdtRotationMatrix[1, 0]))
  pdtEulerAngle[0] = np.arctan2((fSinRoll * pdtRotationMatrix[0, 2]) - (fCosRoll * pdtRotationMatrix[1, 2]), (-fSinRoll * pdtRotationMatrix[0, 1]) + (fCosRoll * pdtRotationMatrix[1, 1]))
  return pdtEulerAngle

def CvtEulerAngleToRotationMatrix(ptrEulerAngle):
  ptrSinAngle = np.sin(ptrEulerAngle)
  ptrCosAngle = np.cos(ptrEulerAngle)
  ptrRotationMatrix = np.zeros((3, 3))
  ptrRotationMatrix[0, 0] = ptrCosAngle[2] * ptrCosAngle[1]
  ptrRotationMatrix[0, 1] = ptrCosAngle[2] * ptrSinAngle[1] * ptrSinAngle[0] - ptrSinAngle[2] * ptrCosAngle[0]
  ptrRotationMatrix[0, 2] = ptrCosAngle[2] * ptrSinAngle[1] * ptrCosAngle[0] + ptrSinAngle[2] * ptrSinAngle[0]
  ptrRotationMatrix[1, 0] = ptrSinAngle[2] * ptrCosAngle[1]
  ptrRotationMatrix[1, 1] = ptrSinAngle[2] * ptrSinAngle[1] * ptrSinAngle[0] + ptrCosAngle[2] * ptrCosAngle[0]
  ptrRotationMatrix[1, 2] = ptrSinAngle[2] * ptrSinAngle[1] * ptrCosAngle[0] - ptrCosAngle[2] * ptrSinAngle[0]
  ptrRotationMatrix[2, 0] = -ptrSinAngle[1]
  ptrRotationMatrix[2, 1] = ptrCosAngle[1] * ptrSinAngle[0]
  ptrRotationMatrix[2, 2] = ptrCosAngle[1] * ptrCosAngle[0]
  return ptrRotationMatrix

