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

        // Restore the rest rotation, and figure out which way the thrust goes.
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
        var lever = m_gimbalTransform.position - vessel.findWorldCenterOfMass();
        var pitchVector = RotateVesselAbout(lever, vesselXform.right);
        var yawVector = RotateVesselAbout(lever, vesselXform.forward);
        var rollVector = RotateVesselAbout(lever, vesselXform.up);

        // Find the direction of thrust to achieve just rotation.  That's the reverse of which way to move the engine part.
        var thrustControlVector = pitchVector * (-s.pitch)
            - yawVector * s.yaw
            - rollVector * s.roll;

        // Gimbal range 1 => 5% of the influence is gimballing, 95% is going down.
        // Seems to be about the same as whatever KSP does stock.
        var thrustVector = thrustControlVector * (5 * gimbalRange)
            + baseThrustVector * (100 - 5 * gimbalRange);
        thrustVector.Normalize();
        m_gimbalTransform.rotation = Quaternion.LookRotation(thrustVector);
    }

    /// <summary>
    /// Return a direction to thrust in, in order to rotate CCW about the axis.
    /// </summary>
    private Vector3d RotateVesselAbout (Vector3 lever, Vector3 axis)
    {
        // Point the gimbal this way in global space to rotate CCW around the axis.
        // The vector is scaled by the sin of the angle, so that we mostly ignore
        // control inputs that have no torque.
        return Vector3d.Cross(lever.normalized, axis.normalized);
    }
}