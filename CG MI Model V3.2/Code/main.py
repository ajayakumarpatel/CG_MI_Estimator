from CG_Estimator import cg_estimator
from MI_Estimator import mi_estimator

if __name__ == "__main__":
    b1_mass = (3804.442, 8370.318, 2500.022)     ##ATF, LOX, Dry
    b2_mass = (3804.442, 8370.318, 2500.022)
    cb_mass = (3804.442, 8370.318, 2470)
    s2_mass = (434.578, 1086.036, 400, 5, 148.895)     ##ATF, LOX, Dry, payload, PLF

    net_cg_x, net_cg_y, net_cg_z = cg_estimator(B1_mass = b1_mass,
                                           B2_mass = b2_mass,
                                           CB_mass = cb_mass,
                                           US_mass = s2_mass, 
                                           is_b1 = True, is_b2 = True, is_core_booster = True, is_PLF = True, is_upper_stage = True)

    print(net_cg_x, net_cg_y, net_cg_z)

    
    cg_nose = (net_cg_x, net_cg_y, net_cg_z)

    result = mi_estimator(b1_mass, b2_mass, cb_mass, s2_mass, cg_nose,
                          is_b1 = True, is_b2 = True, core_booster = True, is_PLF = True, second_stage = True)
    
    print(result)


