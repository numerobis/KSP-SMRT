using System;
using UnityEngine;

/// <summary>
/// Provide a Somewhat More Reasonable Thrust vectoring algorithm than the stock KSP ModuleGimbal.
/// To use, in a part.cfg, replace "ModuleGimbal" with "ModuleSMRTGimbal".
/// </summary>
public class ModuleSMRTGimbal : PartModule
{
    [KSPField]
    public string gimbalTransformName = "thrustTransform";
    private Transform m_gimbalTransform;
    private Quaternion m_restRotation;
    [KSPField]
    public float gimbalRange = 1;
    // in degrees

    [KSPField(guiActive = true, guiName = "Gimbal Locked")]
    private bool gimbalIsLocked = false;

    [KSPAction("Toggle Gimbal")]
    public void ToggleGimbalAction() {
        ToggleGimbal();
    }

    [KSPEvent(guiName = "Toggle Gimbal")]
    public void ToggleGimbal() {
        gimbalIsLocked = !gimbalIsLocked;
    }

    [KSPAction("Free Gimbal")]
    public void FreeGimbalAction() {
        FreeGimbal();
    }

    [KSPEvent(guiName = "Free Gimbal", guiActive = true)]
    public void FreeGimbal() {
        gimbalIsLocked = false;
    }


    [KSPAction("Lock Gimbal")]
    public void LockGimbalAction() {
        LockGimbal();
    }

    [KSPEvent(guiName = "Lock Gimbal", guiActive = true)]
    public void LockGimbal() {
        gimbalIsLocked = true;
    }

    public override void OnStart (StartState state)
    {
        base.OnStart(state);
        m_gimbalTransform = part.FindModelTransform(gimbalTransformName);
        m_restRotation = m_gimbalTransform.localRotation;
    }

    public override void OnActive ()
    {
        base.OnActive();
    }

    public override void OnInactive ()
    {
        base.OnInactive();
    }

    public override void OnFixedUpdate ()
    {
        base.OnFixedUpdate();

        if (gimbalIsLocked)
            return;

        var s = vessel.ctrlState;

        // Restore the rest rotation, and figure out which way the thrust goes.
        m_gimbalTransform.localRotation = m_restRotation;
        var baseThrustVector = m_gimbalTransform.TransformDirection(new Vector3(0, 0, 1));

        // Now figure out which way each control input wants to go.
        var vesselXform = vessel.GetTransform();
        var pitchVector = RotateVesselAbout(-vesselXform.right);
        var yawVector = RotateVesselAbout(-vesselXform.forward);
        var rollVector = RotateVesselAbout(-vesselXform.up);

        // Find the direction of thrust.
        // Gimbal range 1 => 5% of the influence is gimballing, 95% is going down.
        // Seems to be about the same as whatever KSP does stock.
        var thrustVector = (pitchVector * s.pitch + yawVector * s.yaw + rollVector * s.roll) * (5 * gimbalRange)
            + baseThrustVector * (100 - 5 * gimbalRange);
        thrustVector.Normalize();
        m_gimbalTransform.rotation = Quaternion.LookRotation(thrustVector);
    }

    /// <summary>
    /// Return a direction to thrust in, in order to rotate CCW about the axis.
    /// </summary>
    private Vector3d RotateVesselAbout (Vector3 axis)
    {
        var lever = m_gimbalTransform.position - vessel.CoM;

        // Point the gimbal this way in global space to rotate CCW around the axis.
        // The vector is scaled by the sin of the angle, so that we mostly ignore
        // control inputs that have no torque.
        return Vector3d.Cross(lever.normalized, axis.normalized);
    }
}