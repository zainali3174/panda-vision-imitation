import numpy as np
import pinocchio as pin

EE_FRAME = "panda_link8"

Q_PREFERRED = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
JOINT_WEIGHTS = np.array([1.0, 1.0, 1.0, 1.5, 1.0, 1.0, 1.0])

_model = None
_data = None
_ee_frame_id = None


def load_model(urdf_path):
    global _model, _data, _ee_frame_id
    _model = pin.buildModelFromUrdf(urdf_path)
    _data = _model.createData()
    _ee_frame_id = _model.getFrameId(EE_FRAME)
    return _model


def forward_kinematics(q):
    pin.forwardKinematics(_model, _data, q)
    pin.updateFramePlacement(_model, _data, _ee_frame_id)
    return _data.oMf[_ee_frame_id]


def compute_jacobian(q):
    pin.computeJointJacobians(_model, _data, q)
    pin.updateFramePlacements(_model, _data)
    J = pin.getFrameJacobian(_model, _data, _ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    return J


def pose_error(current_se3, target_pos, target_rot):
    pos_err = target_pos - current_se3.translation
    rot_err_matrix = target_rot @ current_se3.rotation.T
    rot_err = pin.log3(rot_err_matrix)
    return np.concatenate([pos_err, rot_err])


def solve_ik(target_pos, target_rot, q_init=None, max_iters=200,
             pos_tol=1e-4, rot_tol=1e-3, elbow_weight=0.1, damping=1e-6,
             max_step=0.2):
    q = q_init.copy() if q_init is not None else Q_PREFERRED.copy()
    q = np.clip(q, _model.lowerPositionLimit, _model.upperPositionLimit)

    for i in range(max_iters):
        ee_pose = forward_kinematics(q)
        err = pose_error(ee_pose, target_pos, target_rot)

        if np.linalg.norm(err[:3]) < pos_tol and np.linalg.norm(err[3:]) < rot_tol:
            return q, True, i

        J = compute_jacobian(q)
        JJt = J @ J.T + damping * np.eye(6)
        dq_primary = J.T @ np.linalg.solve(JJt, err)

        J_pinv = J.T @ np.linalg.inv(JJt)
        N = np.eye(_model.nv) - J_pinv @ J

        posture_error = (Q_PREFERRED - q) * JOINT_WEIGHTS
        dq_secondary = elbow_weight * (N @ posture_error)

        dq = dq_primary + dq_secondary

        step_norm = np.linalg.norm(dq)
        if step_norm > max_step:
            dq = dq * (max_step / step_norm)

        q_new = pin.integrate(_model, q, dq)
        q_new = np.clip(q_new, _model.lowerPositionLimit, _model.upperPositionLimit)
        q = q_new

    return q, False, max_iters


def get_joint_names():
    return list(_model.names)[1:]
