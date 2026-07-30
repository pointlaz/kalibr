from __future__ import print_function #handle print in 2.x python
import sm
from sm import PlotCollection
from kalibr_common import ConfigReader as cr
import aslam_cv as acv
import aslam_cameras_april as acv_april
import aslam_cv_backend as acvb
import aslam_backend as aopt
import incremental_calibration as ic
import kalibr_camera_calibration as kcc

from matplotlib.backends.backend_pdf import PdfPages
import mpl_toolkits.mplot3d.axes3d as p3
import cv2
import numpy as np
import pylab as pl
import math
import gc
import sys

np.set_printoptions(suppress=True, precision=8)

#DV group IDs
CALIBRATION_GROUP_ID = 0
TRANSFORMATION_GROUP_ID = 1
LANDMARK_GROUP_ID = 2

class OptimizationDiverged(Exception):
    pass

def validateIntrinsicsLoadCompatibility(camParams, model_name, distortion_name, resolution, topic=None):
    """T17 — hard-fail loudly if loaded intrinsics do not match the expected setup.

    Cross-checks the camera model, distortion model and image resolution declared in the
    loaded intrinsics YAML against the values requested on the command line (--models,
    split on '-') and the actual dataset resolution. radtan and equidistant both take four
    distortion coefficients, so without this check a wrong file would load silently.

    Args:
        camParams:       kalibr_common.ConfigReader.CameraParameters for one camera.
        model_name:      expected camera-model token ('pinhole', 'omni', 'eucm', 'ds').
        distortion_name: expected distortion-model token ('radtan', 'equidistant',
                         'fov', 'none').
        resolution:      (width, height) from the dataset.
        topic:           image topic, for clearer error messages (optional).

    Returns:
        True on success.

    Raises:
        ValueError: on any camera-model / distortion-model / resolution mismatch.
    """
    where = " for topic {0}".format(topic) if topic is not None else ""

    #The --models distortion token differs from the canonical name written to the YAML for
    #the equidistant model ('equi' on the CLI vs 'equidistant' in the file, per
    #CameraUtils.saveChainParametersYaml / ConfigReader.checkDistortion). Normalize so a
    #correctly-matched pair is not rejected. Camera-model tokens ('pinhole','omni','eucm',
    #'ds') already match the YAML verbatim.
    distortion_alias = {'equi': 'equidistant'}
    expected_distortion = distortion_alias.get(distortion_name, distortion_name)

    loaded_model, _ = camParams.getIntrinsics()
    loaded_distortion, _ = camParams.getDistortion()
    loaded_resolution = camParams.getResolution()

    if loaded_model != model_name:
        raise ValueError(
            "Fixed-intrinsics camera model mismatch{0}: loaded intrinsics file specifies "
            "camera_model '{1}' but the command line (--models) requested '{2}'.".format(
                where, loaded_model, model_name))

    if loaded_distortion != expected_distortion:
        raise ValueError(
            "Fixed-intrinsics distortion model mismatch{0}: loaded intrinsics file "
            "specifies distortion_model '{1}' but the command line (--models) requested "
            "'{2}'.".format(where, loaded_distortion, distortion_name))

    #getResolution() returns [width, height]; resolution is a (width, height) tuple.
    if list(loaded_resolution) != list(resolution):
        raise ValueError(
            "Fixed-intrinsics resolution mismatch{0}: loaded intrinsics file specifies "
            "resolution {1} but the dataset resolution is {2}.".format(
                where, list(loaded_resolution), list(resolution)))

    return True

def safetyCheckFixedIntrinsics(cameras, atol=1e-9):
    """T16 — pre-solve safety net for frozen intrinsics.

    Immediately before the joint optimization, for every camera whose intrinsics are
    frozen this re-injects the originally-loaded projection + distortion parameters and
    asserts (np.allclose, tight tolerance) that the live geometry now matches them. If any
    earlier stage (linear init, per-camera BA, stereoCalibrate, solveFullBatch, divergence
    restart) has silently drifted a fixed camera, this fails loudly here instead of
    producing a wrong result. Cameras that are not frozen are left untouched.

    Args:
        cameras: list of CameraGeometry objects.
        atol:    absolute tolerance for the np.allclose comparison (default 1e-9).

    Raises:
        AssertionError: if a fixed camera's parameters cannot be held at the loaded values.
    """
    for cam_id, cam in enumerate(cameras):
        if not getattr(cam, "fixIntrinsics", False):
            continue

        if cam.loaded_projection is None or cam.loaded_distortion is None:
            raise AssertionError(
                "Camera {0} is flagged fixIntrinsics but has no stored loaded parameters; "
                "initGeometryFromParameters() must run before the safety check.".format(cam_id))

        proj = cam.geometry.projection()
        expected_proj = np.asarray(cam.loaded_projection, dtype=float).flatten()
        expected_dist = np.asarray(cam.loaded_distortion, dtype=float).flatten()

        #diagnostic: report if an upstream stage drifted a supposedly-frozen camera before
        #we correct it (indicates a missed freeze site; the joint solve stays frozen by T11).
        pre_proj = np.asarray(proj.getParameters(), dtype=float).flatten()
        pre_dist = np.asarray(proj.distortion().getParameters(), dtype=float).flatten()
        if not (np.allclose(pre_proj, expected_proj, atol=atol) and np.allclose(pre_dist, expected_dist, atol=atol)):
            sm.logWarn("Fixed cam{0} intrinsics drifted before the joint solve and will be "
                       "re-applied: projection {1} -> {2}, distortion {3} -> {4}.".format(
                           cam_id, pre_proj, expected_proj, pre_dist, expected_dist))

        #re-inject the canonical loaded values
        proj.setParameters(np.array(cam.loaded_projection, dtype=float).reshape(-1, 1))
        proj.distortion().setParameters(np.array(cam.loaded_distortion, dtype=float).reshape(-1, 1))

        #read back and assert numeric equality (NOT bit-for-bit)
        readback_proj = np.asarray(proj.getParameters(), dtype=float).flatten()
        readback_dist = np.asarray(proj.distortion().getParameters(), dtype=float).flatten()

        if not np.allclose(readback_proj, expected_proj, atol=atol):
            raise AssertionError(
                "Camera {0} projection intrinsics drifted despite fixIntrinsics: expected "
                "{1}, got {2}.".format(cam_id, expected_proj, readback_proj))
        if not np.allclose(readback_dist, expected_dist, atol=atol):
            raise AssertionError(
                "Camera {0} distortion coefficients drifted despite fixIntrinsics: expected "
                "{1}, got {2}.".format(cam_id, expected_dist, readback_dist))

        sm.logInfo("Safety check passed for cam{0}: intrinsics frozen and re-applied "
                   "(projection={1}, distortion={2}).".format(cam_id, expected_proj, expected_dist))

class CameraGeometry(object):
    def __init__(self, cameraModel, targetConfig, dataset, geometry=None, verbose=False):
        self.dataset = dataset
        
        self.model = cameraModel
        if geometry is None:
            self.geometry = cameraModel.geometry()
        
        if not type(self.geometry) == cameraModel.geometry:
            raise RuntimeError("The type of geometry passed in \"%s\" does not match the model type \"%s\"" % (type(geometry),type(cameraModel.geometry)))
        
        #create the design variables
        self.dv = cameraModel.designVariable(self.geometry)
        self.isGeometryInitialized = False
        #when True, the intrinsics (projection + distortion) are held fixed and must not
        #be re-estimated. Defaults to False; only initGeometryFromParameters() sets it True.
        #Downstream mutation sites (T11-T21) check this flag; introducing it here so those
        #checks are always safe regardless of how the geometry was initialized.
        self.fixIntrinsics = False
        #loaded intrinsics are stashed here by initGeometryFromParameters() so the
        #pre-solve safety net (safetyCheckFixedIntrinsics) can re-inject and assert them.
        self.loaded_projection = None
        self.loaded_distortion = None
        #T11: honor the freeze flag when first toggling design-variable activity. At
        #construction time fixIntrinsics is always False (only initGeometryFromParameters()
        #flips it True afterwards), so in practice this is the default active setup; the
        #conditional is kept for symmetry with every other setDvActiveStatus site.
        if self.fixIntrinsics:
            self.setDvActiveStatus(False, False, False)
        else:
            self.setDvActiveStatus(True, True, False)

        #create target detector
        self.ctarget = TargetDetector(targetConfig, self.geometry, showCorners=verbose)

    def setDvActiveStatus(self, projectionActive, distortionActive, shutterActice):
        self.dv.projectionDesignVariable().setActive(projectionActive)
        self.dv.distortionDesignVariable().setActive(distortionActive)
        self.dv.shutterDesignVariable().setActive(shutterActice)

    def initGeometryFromObservations(self, observations):
        #When intrinsics are frozen (initGeometryFromParameters was used) we must NOT
        #re-estimate them here: neither the linear focal-length guess (T12) nor the
        #per-camera bundle adjustment (T13) may run, as both mutate projection/distortion
        #in place. This guard also makes the divergence-restart path (T21) safe if a fixed
        #camera ever reaches it. A fixed camera is already initialized, so report success.
        if self.fixIntrinsics:
            sm.logInfo("Intrinsics are frozen for cam with topic {0}; skipping focal-length "
                       "guess and per-camera bundle adjustment.".format(self.dataset.topic))
            self.isGeometryInitialized = True
            return True

        #obtain focal length guess
        success = self.geometry.initializeIntrinsics(observations)
        if not success:
            sm.logError("initialization of focal length for cam with topic {0} failed  ".format(self.dataset.topic))

        #in case of an omni model, first optimize over intrinsics only
        #(--> catch most of the distortion with the projection model)
        if self.model == acvb.DistortedOmni:
            success = kcc.calibrateIntrinsics(self, observations, distortionActive=False)
            if not success:
                sm.logError("initialization of intrinsics for cam with topic {0} failed  ".format(self.dataset.topic))

        #optimize for intrinsics & distortion
        success = kcc.calibrateIntrinsics(self, observations)
        if not success:
            sm.logError("initialization of intrinsics for cam with topic {0} failed  ".format(self.dataset.topic))

        self.isGeometryInitialized = success
        return success

    def initGeometryFromParameters(self, camParams, model_name, distortion_name, resolution):
        """Initialize the camera geometry from previously-calibrated intrinsics and
        freeze them, instead of auto-estimating from observations.

        This is the fixed-intrinsics sibling of initGeometryFromObservations(): rather
        than running initializeIntrinsics()/calibrateIntrinsics() on the target
        observations, it injects the loaded projection + distortion parameters directly
        into self.geometry and flags the intrinsics as fixed so that downstream
        estimation steps leave them untouched.

        Args:
            camParams:       a kalibr_common.ConfigReader.CameraParameters object holding
                             the per-camera intrinsics (already extracted from the camera
                             chain by the caller; NOT the chain, NOT a file path).
            model_name:      expected camera-model token parsed from --models for this
                             camera (e.g. 'pinhole', 'omni', 'eucm', 'ds').
            distortion_name: expected distortion-model token (e.g. 'radtan',
                             'equidistant', 'fov', 'none').
            resolution:      (width, height) tuple from the dataset.

        Raises:
            ValueError: if the loaded camera model, distortion model, or resolution do
                        not match the expected values.
        """
        #--- read the loaded parameters (getIntrinsics/getDistortion also validate
        #    internally via checkIntrinsics/checkDistortion in ConfigReader) ---
        loaded_model, intrinsics = camParams.getIntrinsics()
        loaded_distortion, dist_coeffs = camParams.getDistortion()

        #--- validate against the expected (CLI-derived / dataset-derived) values ---
        #T17: cross-check camera model, distortion model and resolution loudly before
        #injecting anything (radtan and equidistant both take 4 coeffs -> a wrong load
        #would otherwise be silent). Raises ValueError on any mismatch.
        validateIntrinsicsLoadCompatibility(
            camParams, model_name, distortion_name, resolution, topic=self.dataset.topic)

        #--- inject the loaded parameters into the live geometry ---
        #The intrinsics list is stored (by CameraParameters.setIntrinsics) in exactly the
        #same order the projection model expects, so it maps directly onto
        #projection().setParameters() without reordering:
        #   pinhole -> [fu, fv, cu, cv]
        #   omni    -> [xi, fu, fv, cu, cv]
        #   eucm    -> [alpha, beta, fu, fv, cu, cv]
        #   ds      -> [xi, alpha, fu, fv, cu, cv]
        #This is the exact inverse of saveChainParametersYaml() in CameraUtils.py.
        #Distortion coeffs map directly onto distortion().setParameters():
        #   radtan -> [k1, k2, p1, p2]; equidistant -> [k1, k2, k3, k4];
        #   fov -> [w]; none -> [].
        #setParameters is bound to an Eigen Nx1 matrix, so reshape to (-1, 1).
        proj_params = np.array(intrinsics, dtype=float).reshape(-1, 1)
        dist_params = np.array(dist_coeffs, dtype=float).reshape(-1, 1)
        proj = self.geometry.projection()
        proj.setParameters(proj_params)
        proj.distortion().setParameters(dist_params)

        #--- record state ---
        self.isGeometryInitialized = True
        self.fixIntrinsics = True
        self.model_name = model_name
        self.distortion_name = distortion_name
        #T16: keep the canonical loaded values so the pre-solve safety net can re-inject
        #and assert them. Store copies so later in-place mutations can't corrupt them.
        self.loaded_projection = proj_params.copy()
        self.loaded_distortion = dist_params.copy()

        return True

class TargetDetector(object):
    def __init__(self, targetConfig, cameraGeometry, showCorners=False, showReproj=False, showOneStep=False):
        self.targetConfig = targetConfig
        
        #initialize the calibration target
        targetParams = targetConfig.getTargetParams()
        targetType = targetConfig.getTargetType()

        if targetType == 'checkerboard':
            options = acv.CheckerboardOptions()
            options.filterQuads = True
            options.normalizeImage = True
            options.useAdaptiveThreshold = True        
            options.performFastCheck = False
            options.windowWidth = 5            
            options.showExtractionVideo = showCorners
            
            self.grid = acv.GridCalibrationTargetCheckerboard(targetParams['targetRows'], 
                                                              targetParams['targetCols'], 
                                                              targetParams['rowSpacingMeters'], 
                                                              targetParams['colSpacingMeters'], 
                                                              options)
        elif targetType == 'circlegrid':
            options = acv.CirclegridOptions()
            options.showExtractionVideo = showCorners
            options.useAsymmetricCirclegrid = targetParams['asymmetricGrid']
            
            self.grid = acv.GridCalibrationTargetCirclegrid(targetParams['targetRows'],
                                                           targetParams['targetCols'], 
                                                           targetParams['spacingMeters'], 
                                                           options)
         
        elif targetType == 'aprilgrid':
            options = acv_april.AprilgridOptions()
            #enforce more than one row --> pnp solution can be bad if all points are almost on a line...
            options.minTagsForValidObs = int( np.max( [targetParams['tagRows'], targetParams['tagCols']] ) + 1 )
            options.showExtractionVideo = showCorners
            
            self.grid = acv_april.GridCalibrationTargetAprilgrid(targetParams['tagRows'], 
                                                                 targetParams['tagCols'], 
                                                                 targetParams['tagSize'], 
                                                                 targetParams['tagSpacing'], 
                                                                 options)
        else:
            RuntimeError('Unknown calibration target type!')

        options = acv.GridDetectorOptions() 
        options.imageStepping = showOneStep
        options.plotCornerReprojection = showReproj
        options.filterCornerOutliers = False
        
        self.detector = acv.GridDetector(cameraGeometry, self.grid, options)

class CalibrationTarget(object):
    def __init__(self, target, estimateLandmarks=False):
        self.target = target
        # Create design variables and expressions for all target points.
        P_t_dv = []
        P_t_ex = []
        for i in range(0,self.target.size()):
            p_t_dv = aopt.HomogeneousPointDv(sm.toHomogeneous(self.target.point(i)));
            p_t_dv.setActive(estimateLandmarks)
            p_t_ex = p_t_dv.toExpression()
            P_t_dv.append(p_t_dv)
            P_t_ex.append(p_t_ex)
        self.P_t_dv = P_t_dv
        self.P_t_ex = P_t_ex
    def getPoint(self,i):
        return P_t_ex[i]

class CalibrationTargetOptimizationProblem(ic.CalibrationOptimizationProblem):        
    @classmethod
    def fromTargetViewObservations(cls, cameras, target, baselines, timestamp, T_tc_guess, rig_observations, useBlakeZissermanMest=True):
        rval = CalibrationTargetOptimizationProblem()        

        #store the arguements in case we want to rebuild a modified problem
        rval.cameras = cameras
        rval.target = target
        rval.baselines = baselines
        rval.timestamp = timestamp
        rval.T_tc_guess = T_tc_guess
        rval.rig_observations = rig_observations
        
        # 1. Create a design variable for this pose
        T_target_camera = T_tc_guess
        
        rval.dv_T_target_camera = aopt.TransformationDv(T_target_camera)
        for i in range(0, rval.dv_T_target_camera.numDesignVariables()):
            rval.addDesignVariable( rval.dv_T_target_camera.getDesignVariable(i), TRANSFORMATION_GROUP_ID)
        
        #2. add all baselines DVs
        for baseline_dv in baselines:
            for i in range(0, baseline_dv.numDesignVariables()):
                rval.addDesignVariable(baseline_dv.getDesignVariable(i), CALIBRATION_GROUP_ID)
        
        #3. add landmark DVs
        for p in target.P_t_dv:
            rval.addDesignVariable(p,LANDMARK_GROUP_ID)
        
        #4. add camera DVs
        for camera in cameras:
            if not camera.isGeometryInitialized:
                raise RuntimeError('The camera geometry is not initialized. Please initialize with initGeometry() or initGeometryFromDataset()')
            #T11: the joint solve is the last mutation site. The design variables are still
            #added to the problem (they participate in the reprojection error expressions),
            #but for a fixed camera projection+distortion are marked inactive so the solver
            #leaves the frozen intrinsics untouched; only extrinsics/target poses move.
            if getattr(camera, "fixIntrinsics", False):
                camera.setDvActiveStatus(False, False, False)
            else:
                camera.setDvActiveStatus(True, True, False)
            rval.addDesignVariable(camera.dv.distortionDesignVariable(), CALIBRATION_GROUP_ID)
            rval.addDesignVariable(camera.dv.projectionDesignVariable(), CALIBRATION_GROUP_ID)
            rval.addDesignVariable(camera.dv.shutterDesignVariable(), CALIBRATION_GROUP_ID)
        
        #4.add all observations for this view
        cams_in_view = set()
        rval.rerrs=dict()
        rerr_cnt=0
        for cam_id, obs in rig_observations:
            camera = cameras[cam_id]
            cams_in_view.add(cam_id)
            
            #add reprojection errors
            #build baseline chain (target->cam0->baselines->camN)                
            T_cam0_target = rval.dv_T_target_camera.expression.inverse()
            T_camN_calib = T_cam0_target
            for idx in range(0, cam_id):
                T_camN_calib =  baselines[idx].toExpression() * T_camN_calib
            
            # \todo pass in the detector uncertainty somehow.
            cornerUncertainty = 1.0
            R = np.eye(2) * cornerUncertainty * cornerUncertainty
            invR = np.linalg.inv(R)
            
            rval.rerrs[cam_id] = list()
            for i in range(0,len(target.P_t_ex)):
                p_target = target.P_t_ex[i]
                valid, y = obs.imagePoint(i)
                if valid:
                    rerr_cnt+=1
                    # Create an error term.
                    rerr = camera.model.reprojectionError(y, invR, T_camN_calib * p_target, camera.dv)
                    rerr.idx = i
                    
                    #add blake-zisserman mest
                    if useBlakeZissermanMest:
                        mest = aopt.BlakeZissermanMEstimator( 2.0 )
                        rerr.setMEstimatorPolicy(mest)
                    rval.addErrorTerm(rerr)
                    rval.rerrs[cam_id].append(rerr)
                else:
                    rval.rerrs[cam_id].append(None)

        sm.logDebug("Adding a view with {0} cameras and {1} error terms".format(len(cams_in_view), rerr_cnt))
        return rval

def removeCornersFromBatch(batch, camId_cornerIdList_tuples, useBlakeZissermanMest=True):
    #translate (camid,obs) tuple to dict
    obsdict=dict()
    for cidx, obs in batch.rig_observations:
        obsdict[cidx]=obs
       
    #disable the corners
    hasCornerRemoved=False
    for cidx, removelist in camId_cornerIdList_tuples:
        for corner_id in removelist: 
            obsdict[cidx].removeImagePoint(corner_id)
            hasCornerRemoved=True
    assert hasCornerRemoved, "need to remove at least one corner..."
    
    #rebuild problem
    new_problem = CalibrationTargetOptimizationProblem.fromTargetViewObservations(batch.cameras, 
                                                                                  batch.target, 
                                                                                  batch.baselines, 
                                                                                  batch.timestamp, 
                                                                                  batch.T_tc_guess, 
                                                                                  batch.rig_observations,
                                                                                  useBlakeZissermanMest=useBlakeZissermanMest)

    return new_problem
        
class CameraCalibration(object):
    def __init__(self, cameras, baseline_guesses, estimateLandmarks=False, verbose=False, useBlakeZissermanMest=True):
        self.cameras = cameras
        self.useBlakeZissermanMest = useBlakeZissermanMest
        #create the incremental estimator
        self.estimator = ic.IncrementalEstimator(CALIBRATION_GROUP_ID)
        self.linearSolverOptions = self.estimator.getLinearSolverOptions()
        self.optimizerOptions = self.estimator.getOptimizerOptions()
        self.target = CalibrationTarget(cameras[0].ctarget.detector.target(), estimateLandmarks=estimateLandmarks)
        self.initializeBaselineDVs(baseline_guesses)
        #storage for the used views
        self.views = list()
        
    def initializeBaselineDVs(self, baseline_guesses):
        self.baselines = list()
        for baseline_idx in range(0, len(self.cameras)-1): 
            self.baselines.append( aopt.TransformationDv(baseline_guesses[baseline_idx]) )
            
    def getBaseline(self, i):
        return self.baselines[i]
    
    def addTargetView(self, timestamp, rig_observations, T_tc_guess, force=False):
        #create the problem for this batch and try to add it 
        batch_problem = CalibrationTargetOptimizationProblem.fromTargetViewObservations(self.cameras, self.target, self.baselines, timestamp, T_tc_guess, rig_observations, useBlakeZissermanMest=self.useBlakeZissermanMest)
        self.estimator_return_value = self.estimator.addBatch(batch_problem, force)
        
        if self.estimator_return_value.numIterations >= self.optimizerOptions.maxIterations:
            sm.logError("Did not converge in maxIterations... restarting...")
            raise OptimizationDiverged
        
        success = self.estimator_return_value.batchAccepted
        if success:
            sm.logDebug("The estimator accepted this batch")
            self.views.append(batch_problem)
        else:
            sm.logDebug("The estimator did not accept this batch")
        return success

