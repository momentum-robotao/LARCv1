import glob
import os

import cv2 as cv
import numpy as np


def calibrate(showPics=True):
    # Read image
    root = os.getcwd()
    imagesDir = os.path.join(root, "auxiliar_codes/calibrate_camera/images")
    imgPathList = glob.glob(os.path.join(imagesDir, "*.png"))

    if not imgPathList:
        return

    # Initialize
    nRows = 7
    nCols = 7
    termCriteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    worldPtsCur = np.array(
        [[x / 100, y / 100, 0] for x in range(nCols) for y in range(nRows)],
        dtype=np.float32,
    )

    worldPtsList = []
    imgPtsList = []

    # Find Corners
    for curImgPath in imgPathList:
        if "ok" not in curImgPath:
            continue
        imgBGR = cv.imread(curImgPath)
        imgGray = cv.cvtColor(imgBGR, cv.COLOR_BGR2GRAY)

        # Improve contrast
        # imgGray = cv.equalizeHist(imgGray)

        cornersFound, cornersOrg = cv.findChessboardCorners(
            imgGray, (nRows, nCols), None
        )
        print(curImgPath, cornersFound)
        if cornersFound:
            worldPtsList.append(worldPtsCur)
            cornersRefined = cv.cornerSubPix(
                imgGray, cornersOrg, (11, 11), (-1, -1), termCriteria
            )
            imgPtsList.append(cornersRefined)
            if showPics:
                cv.drawChessboardCorners(
                    imgBGR, (nRows, nCols), cornersRefined, cornersFound
                )
                cv.imshow(curImgPath[-12:].rstrip(".png"), imgBGR)
                cv.waitKey(300)
    cv.destroyAllWindows()

    # Calibrate
    repError, camMatrix, distCoeff, rvecs, tvecs = cv.calibrateCamera(
        worldPtsList, imgPtsList, imgGray.shape[::-1], None, None
    )
    print(f"Camera matrix: {camMatrix}")
    print(f"Reproj Error (pixels): {repError:.4f}")

    # Save calibration parameters
    curFolder = os.path.dirname(os.path.abspath(__file__))
    paramPath = os.path.join(curFolder, "calibration.npz")
    np.savez(
        paramPath,
        repError=repError,
        camMatrix=camMatrix,
        distCoeff=distCoeff,
        rvecs=rvecs,
        tvecs=tvecs,
    )

    return camMatrix, distCoeff


if __name__ == "__main__":
    calibrate()
