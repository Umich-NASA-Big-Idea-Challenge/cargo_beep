import numpy as np

def design_segway_controller(robot_params):
    """
    Design a state-space controller for a balancing robot based on specific parameters
    Using only NumPy (no control library)
    
    Parameters:
    -----------
    robot_params : dict
        Dictionary containing the robot's physical parameters
        
    Returns:
    --------
    dict: Controller matrices and design parameters
    """
    # Extract robot parameters
    g = robot_params['g']
    m_wh = robot_params['m_wh']
    m_pend = robot_params['m_pend']
    L = robot_params['L']
    r_wh = robot_params['r_wh']
    J_pend = robot_params['J_pend']
    J_wh = robot_params['J_wh']
    J_rotor = robot_params['J_rotor']
    N = robot_params['N']
    
    print("Designing controller for robot with:")
    print(f"  Wheel mass: {m_wh} kg")
    print(f"  Body mass: {m_pend} kg")
    print(f"  COM height: {L} m")
    print(f"  Wheel radius: {r_wh} m")
    
    # Step 1: Linearize the Segway Model
    # Compute linearized model matrices at the upright equilibrium (theta = 0)
    
    # D matrix (inertia matrix) at theta = 0
    D = np.zeros((2, 2))
    D[0, 0] = (J_pend*m_pend + J_pend*m_wh + (m_pend**2)*(r_wh**2) + (m_wh**2)*(r_wh**2) + 
              2.0*J_wh*m_pend + 2.0*J_wh*m_wh + (L**2)*(m_pend**2) + 
              2.0*m_pend*m_wh*(r_wh**2) + 2.0*L*r_wh*(m_pend**2) + 
              2.0*L*m_pend*m_wh*r_wh) / (m_pend + m_wh)
    D[0, 1] = 2.0*J_wh + m_pend*(r_wh**2) + m_wh*(r_wh**2) + L*m_pend*r_wh
    D[1, 0] = D[0, 1]  # D is symmetric
    D[1, 1] = 2.0*J_wh + m_pend*(r_wh**2) + m_wh*(r_wh**2) + 2.0*J_rotor*(N**2)
    
    # Jacobian of G (gravity vector) at theta = 0
    JacG = np.zeros((2, 2))
    JacG[0, 0] = -L*g*m_pend  # This is the partial derivative of G[0] with respect to theta
    
    # Control input matrix B
    B = np.zeros((2, 1))
    B[1, 0] = 2*N  # Motor torque input affects phi (wheel angle) via gear ratio
    
    # Build the linearized state variable model A matrix
    A21 = -np.linalg.solve(D, JacG)  # A21 = -D⁻¹ * JacG
    
    A = np.zeros((4, 4))
    A[0:2, 2:4] = np.eye(2)  # Top-right block is identity (dx/dt = v)
    A[2:4, 0:2] = A21        # Bottom-left block is A21
    
    # Build the B matrix for the state variable model
    b = np.zeros((4, 1))
    b[2:4, :] = np.linalg.solve(D, B)  # b[2:4] = D⁻¹ * B
    
    print("\nLinearized State Space Model:")
    print("A matrix:")
    print(A)
    print("\nb matrix:")
    print(b)
    
    # Step 2: Inner Loop PD Controller Design for Angle Control
    # Extract transfer function parameters from the A matrix
    k0 = b[2, 0]  # This is the numerator coefficient for theta response
    a0 = A[2, 0]  # This is the constant term in the denominator
    a1 = 0        # There's no damping in the A matrix for this model
    
    print(f"\nTransfer Function Parameters: k0 = {k0}, a0 = {a0}, a1 = {a1}")
    
    # Design parameters for the controller
    zeta = 0.7    # Damping ratio (0.7 gives about 5% overshoot)
    Ts = 0.7      # Desired settling time
    wn = 3.0/(zeta*Ts)  # Calculate natural frequency from settling time
    
    # Calculate controller gains using the textbook formulas
    Kp = (wn**2 - a0)/k0  # Proportional gain
    Kd = (2*zeta*wn - a1)/k0  # Derivative gain
    
    # Calculate zeros from gains - important for stabilization
    # Use a reasonable default if calculation would cause division by zero
    tau_d = 0.01  # Dirty derivative time constant
    
    # We need to ensure these gains will stabilize the system
    # For inverted pendulum, Kp typically needs to be negative
    # and Kd needs proper sign to add damping
    
    # Check if gains seem reasonable for stabilization
    if a0 > 0:  # Unstable open-loop system (pendulum)
        # For unstable systems, correct sign is crucial
        if k0 * Kp > 0:  # Wrong stabilizing direction
            print("Warning: Calculated Kp has wrong sign for stabilization. Adjusting.")
            Kp = -abs(Kp)  # Force negative
    
    # Calculate velocity control gain - can be tuned
    K2 = 0.15  # Start with textbook value, can be adjusted
    
    # Calculate zeros based on controller gains
    if Kd != 0:
        z_angle = abs(Kp/Kd)  # PD controller zero
    else:
        z_angle = 10.0  # Default value if Kd is zero
    
    # Velocity controller zero - can be tuned based on response
    z_vel = 2.722  # Start with textbook value, can be adjusted
    
    # Pre-compensator gain for steady-state tracking
    if a0 + k0*Kp != 0:
        kF = a0/(a0 + k0*Kp)
    else:
        kF = 0.833  # Default value if calculation would cause division by zero
    
    print("\nController Design Parameters:")
    print(f"  Damping ratio (zeta): {zeta}")
    print(f"  Natural frequency (wn): {wn} rad/s")
    print(f"  Settling time (Ts): {Ts} s")
    print(f"  Proportional gain (Kp): {Kp}")
    print(f"  Derivative gain (Kd): {Kd}")
    print(f"  Angle zero (z_angle): {z_angle}")
    print(f"  Velocity zero (z_vel): {z_vel}")
    print(f"  Pre-compensator gain (kF): {kF}")
    
    # Step 3: Construct the state-space matrices using calculated values
    # For a causal implementation with dirty derivative
    
    # Construct state-space matrices using the calculated parameters
    Ac = np.array([
        [-1/tau_d, 0.0, 0.0],       # Dirty derivative state
        [1.0, -z_vel, 0.0],         # Velocity controller zero
        [0.0, 0.0, -z_angle]        # Angle controller zero
    ])
    
    Bc = np.array([
        [1/tau_d, 0.0],             # Input to dirty derivative
        [0.0, 0.0],                 # Velocity error input
        [0.0, z_angle]              # Angle error input
    ])
    
    Cc = np.array([
        [Kd, -K2, Kp]               # Output gains
    ])
    
    Dc = np.array([
        [K2, Kp]                    # Direct feedthrough
    ])
    
    print("\nState Space Controller:")
    print("Ac matrix:")
    print(Ac)
    print("\nBc matrix:")
    print(Bc)
    print("\nCc matrix:")
    print(Cc)
    print("\nDc matrix:")
    print(Dc)
    
    # Return the designed controller
    return {
        'Ac': Ac,
        'Bc': Bc,
        'Cc': Cc,
        'Dc': Dc,
        'K1': Kp,
        'K2': K2,
        'K3': kF,
        'tau_d': tau_d,
        'zeta': zeta,
        'wn': wn,
        'Ts': Ts
    }

def main():
    # Example using custom robot parameters
    robot_params = {
        'g': 9.81,          # Gravity (m/s^2)
        'm_wh': 2.4,        # kg mass of 2 wheels 
        'm_pend': 10.9,     # kg mass of everything that is not the wheels
        'L': 0.125,         # m CoM height
        'r_wh': 0.4,        # m, Wheel radius
        'J_pend': None,     # Will be calculated
        'J_wh': 2.4/2 * (0.4**2),  # kg m^2 moment of inertia of each wheel
        'J_rotor': 1e-4,    # kg m^2 per rotor, one for each wheel
        'N': 1              # gear ratio between each motor rotor and wheel
    }
    
    # Calculate J_pend based on pendulum mass and length
    robot_params['J_pend'] = robot_params['m_pend'] * (robot_params['L']**2) / 6
    
    # Design the controller using just NumPy
    controller = design_segway_controller(robot_params)
    
    # Display the result for easy implementation
    print("\nFor implementation, copy these matrices into your controller:")
    print("self.Ac = np.array([")
    for row in controller['Ac']:
        print(f"    [{', '.join(f'{val:.6f}' for val in row)}],")
    print("])")
    
    print("\nself.Bc = np.array([")
    for row in controller['Bc']:
        print(f"    [{', '.join(f'{val:.6f}' for val in row)}],")
    print("])")
    
    print("\nself.Cc = np.array([")
    for row in controller['Cc']:
        print(f"    [{', '.join(f'{val:.6f}' for val in row)}],")
    print("])")
    
    print("\nself.Dc = np.array([")
    for row in controller['Dc']:
        print(f"    [{', '.join(f'{val:.6f}' for val in row)}],")
    print("])")
    
    print("\nController Gains:")
    print(f"K1 (Angle) = {controller['K1']:.6f}")
    print(f"K2 (Velocity) = {controller['K2']:.6f}")
    print(f"K3 (Precompensator) = {controller['K3']:.6f}")

if __name__ == "__main__":
    main()