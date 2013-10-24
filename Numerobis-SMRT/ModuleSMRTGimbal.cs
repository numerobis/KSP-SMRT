using System;
using UnityEngine;

/// <summary>
/// Provide a Somewhat More Reasonable Thrust vectoring algorithm than the stock KSP ModuleGimbal.
/// To use, in a part.cfg, replace "ModuleGimbal" with "ModuleSMRTGimbal".
/// </summary>
public class ModuleSMRTGimbal : ModuleGimbal
{
    private Transform m_gimbalTransform;
    private Quaternion m_restRotation;

    [KSPAction("Free Gimbal")]
    public void FreeGimbalAction() {
        FreeGimbal();
    }

    [KSPAction("Lock Gimbal")]
    public void LockGimbalAction() {
        LockGimbal();
    }

    public override void OnStart (StartState state)
    {
        base.OnStart(state);
        m_gimbalTransform = part.FindModelTransform(gimbalTransformName);
        m_restRotation = m_gimbalTransform.localRotation;
    }

    public override void OnFixedUpdate ()
    {
        if (gimbalLock)
            return;

        var s = vessel.ctrlState;

        // Restore the rest rotation, and figure out which way the thrust goes in world space.
        m_gimbalTransform.localRotation = m_restRotation;
        var baseThrustVector = m_gimbalTransform.TransformDirection(new Vector3(0, 0, 1));

        // Now figure out which way to move the engine part in order to effect a pitch/yaw/roll moment.
        // The vectors returned are direction scaled by the angle we form; a small angle will be a short
        // vector and thus we won't try to use this input to develop that moment.
        //
        // Note that in vessel space, "forward" means up to the sky if your spacecraft is a plane on the runway,
        // and "up" means down the runway.  The "up" name makes more sense for a rocket on the pad.
        // 
        var vesselXform = vessel.GetTransform();
        var lever = (Vector3d)m_gimbalTransform.position - (Vector3d)vessel.findWorldCenterOfMass();

        var pitchVector = RotateVesselAbout(lever, vesselXform.right, baseThrustVector, s.pitch);
        var yawVector = RotateVesselAbout(lever, vesselXform.forward, baseThrustVector, s.yaw);
        var rollVector = RotateVesselAbout(lever, vesselXform.up, baseThrustVector, s.roll);

        // Find the direction of thrust to achieve just rotation.
        var thrustControlVector = pitchVector + yawVector + rollVector;

        // Clip that vector to the gimbal range
        var thrustVector = ClipToGimbalCone(baseThrustVector, thrustControlVector, 1.0);
        if (thrustVector.sqrMagnitude > 0) {
            thrustVector.Normalize();
            m_gimbalTransform.rotation = Quaternion.LookRotation(thrustVector);
        }
    }

    /// <summary>
    /// Return a direction to thrust in, in order to torque according to the control input.
    /// The control input specifies an amount to rotate in the CW direction.
    /// </summary>
    private Vector3d RotateVesselAbout (Vector3d lever, Vector3d axis, Vector3d neutralPose, float controlInput)
    {
        // Point the gimbal this way in global space to rotate CCW around the axis.
        // The vector is scaled by the sin of the angle; ignore control inputs with
        // zero torque.
        var direction = Vector3d.Cross(lever.normalized, axis.normalized);

        // Compute a max that doesn't exceed the gimbal range.
        // Flip the direction of the control input since it is CW whereas the direction is CCW.
        var clippedDirection = ClipToGimbalCone(neutralPose, direction, -controlInput);
        return clippedDirection;
    }

    /// <summary>
    /// Compute a vector that gimbals a fraction of the way from the neutral pose to the target,
    /// taking into account the maximum gimbal range.  If factor is 1, we point at the target
    /// or the closest point to it within gimbal range; if factor is 0.5, we bisect the angle, etc.s
    /// If factor is negative, flip the target around.
    /// </summary>
    private Vector3d ClipToGimbalCone(Vector3d neutralPose, Vector3d target, double factor) {
        if (factor == 0) {
            return new Vector3d(0, 0, 0);
        }

        // Calculations below require a non-zero target vector.
        var r = target.magnitude;
        if (r < 1e-6) {
            return new Vector3d(0, 0, 0);
        }

        // Work in a world where the factor is positive, to eliminate special cases below.
        if (factor < 0) {
            factor = -factor;
            target = -target;
        }

        // Check if we can point directly at the target, including if target and neutral are
        // parallel.
        var idealDegrees = Vector3d.Angle(neutralPose, target);
        if ( (idealDegrees < 1e-3) || (idealDegrees < gimbalRange && factor > 1.0 - 1e-3)) {
            // Shortcut to avoid numerical error: if we can point at the target, and
            // we want the maximum control authority, point at the target.
            return target;
        }

        // Compute the angle we want to make, taking into account the factor.
        var maxDegrees = Math.Min(gimbalRange, idealDegrees);
        var targetDegrees = maxDegrees * factor;
        var targetRadians = targetDegrees * Math.PI / 180.0;

        // target is n r cos(angle) + n' r sin(angle)
        // we want   n r cos(targetAngle) + n' r sin(targetAngle)
        // where n and n' form an orthonormal basis
        // Computing n' requires orthonormalization, which requires that target isn't
        // parallel and non-zero (we have checked both those conditions already).
        // Switch to Vector3 since Vector3d doesn't actually implement OrthoNormalize.  TODO.
        var nprime = (Vector3)target;
        var n = (Vector3)neutralPose;
        Vector3.OrthoNormalize(ref n, ref nprime);

        var clipped = (r * Math.Cos(targetRadians) * (Vector3d)n) + (r * Math.Sin(targetRadians) * (Vector3d)nprime);
        return clipped;
    }
}