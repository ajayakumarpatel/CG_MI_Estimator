"""
Code Title: Moment of Inertia Estimation Tool
Code Version: V2.0
Developed By: Ajaya Kumar Patel
Release Date: 08-11-2024
CAD Data Version: V3.2

Description:
CGEstimation Tool, a software designed for calculating the centre of gravity across the entire flight duration.
The tool determines the CG as a function of mass, ensuring accurate tracking of CG shifts as propellant is consumed.

Features:
- Horner’s method is used to evaluate the polynomial in linear time.
- Since the number of variables is small, the coefficients are directly hardcoded instead of being read from a file.

Contact:
For feedback or inquiries, please contact: [ajayakumarpatel@agnikul.in]
"""

import numpy as np
import pandas as pd

def evaluate_poly_horner(x, coeffs):
    result = 0
    for coeff in coeffs:
        result = result * x + coeff
    return result

def cg_estimator(B1_mass, B2_mass, CB_mass, US_mass, is_b1 = True, is_b2 = True, is_core_booster = True, is_PLF = True, is_upper_stage = True):
    
    atf_mass_b1, lox_mass_b1, dry_mass_b1 = B1_mass
    atf_mass_b2, lox_mass_b2, dry_mass_b2 = B2_mass
    atf_mass_cb, lox_mass_cb, dry_mass_cb = CB_mass
    atf_mass_us, lox_mass_us, dry_mass_us, payload_mass, PLF_mass = US_mass
    
    net_cg_x_numerator = 0.0
    net_cg_y_numerator = 0.0
    net_cg_z_numerator = 0.0
    
    net_cg_denominator = 0.0
    
    if(is_b1 == True):
        # print('PY')
        coeff_cg_x_atf_b1 = np.array([ 4.03962234e-33, -8.23591306e-29,  7.30425346e-25, -3.69885559e-21,
                                      1.18009395e-17, -2.46881031e-14,  3.41692989e-11, -3.08295720e-08,
                                      1.74400821e-05, -5.77168544e-03,  1.49577726e+00, -1.75503752e+04])
        
        coeff_cg_x_lox_b1 = np.array([ 1.03416665e-36, -4.64219815e-32,  9.06235308e-28, -1.00972755e-23,
                                       7.08316918e-20, -3.25448239e-16,  9.87367163e-13, -1.94607673e-09,
                                       2.38869428e-06, -1.68988023e-03,  9.30297322e-01, -1.28073830e+04])
        
        cg_x_atf_b1 = evaluate_poly_horner(atf_mass_b1, coeff_cg_x_atf_b1)
        cg_y_atf_b1 = 1430
        cg_z_atf_b1 = 0.0
        
        cg_x_lox_b1 = evaluate_poly_horner(lox_mass_b1, coeff_cg_x_lox_b1)
        cg_y_lox_b1 = 1430
        cg_z_lox_b1 = 0.0
        
        cg_x_dry_b1 = -15805.647
        cg_y_dry_b1 = 1428.506
        cg_z_dry_b1 = -8.337

        net_cg_x_numerator += cg_x_atf_b1 * atf_mass_b1 + cg_x_lox_b1 * lox_mass_b1 + cg_x_dry_b1 * dry_mass_b1
        net_cg_y_numerator += cg_y_atf_b1 * atf_mass_b1 + cg_y_lox_b1 * lox_mass_b1 + cg_y_dry_b1 * dry_mass_b1
        net_cg_z_numerator += cg_z_atf_b1 * atf_mass_b1 + cg_z_lox_b1 * lox_mass_b1 + cg_z_dry_b1 * dry_mass_b1
        
        net_cg_denominator += atf_mass_b1 + lox_mass_b1 + dry_mass_b1
        
        ##Booster - 2
    if(is_b2 == True):
        # print('MY')
        coeff_cg_x_atf_b2 = np.array([ 4.03962234e-33, -8.23591306e-29,  7.30425346e-25, -3.69885559e-21,
                                       1.18009395e-17, -2.46881031e-14,  3.41692989e-11, -3.08295720e-08,
                                       1.74400821e-05, -5.77168544e-03,  1.49577726e+00, -1.75503752e+04])
        
        coeff_cg_x_lox_b2 = np.array([1.03411045e-36, -4.64196226e-32,  9.06192971e-28, -1.00968516e-23,
                                      7.08291025e-20, -3.25438326e-16,  9.87343652e-13, -1.94604381e-09,
                                      2.38866956e-06, -1.68987191e-03,  9.30296379e-01, -1.28073830e+04])
        
        cg_x_atf_b2 = evaluate_poly_horner(atf_mass_b2, coeff_cg_x_atf_b2)
        cg_y_atf_b2 = -1430
        cg_z_atf_b2 = 0.0
        
        cg_x_lox_b2 = evaluate_poly_horner(lox_mass_b2, coeff_cg_x_lox_b2)
        cg_y_lox_b2 = -1430
        cg_z_lox_b2 = 0.0
        
        cg_x_dry_b2 = -15805.647
        cg_y_dry_b2 = -1428.506
        cg_z_dry_b2 = 8.337

        net_cg_x_numerator += cg_x_atf_b2 * atf_mass_b2 + cg_x_lox_b2 * lox_mass_b2 + cg_x_dry_b2 * dry_mass_b2
        net_cg_y_numerator += cg_y_atf_b2 * atf_mass_b2 + cg_y_lox_b2 * lox_mass_b2 + cg_y_dry_b2 * dry_mass_b2
        net_cg_z_numerator += cg_z_atf_b2 * atf_mass_b2 + cg_z_lox_b2 * lox_mass_b2 + cg_z_dry_b2 * dry_mass_b2
        
        net_cg_denominator += atf_mass_b2 + lox_mass_b2 + dry_mass_b2
    
    if(is_core_booster == True):
        # print('CB')
        coeff_cg_x_atf_cb = np.array([ 4.03962234e-33, -8.23591306e-29,  7.30425346e-25, -3.69885559e-21,
                                       1.18009395e-17, -2.46881031e-14,  3.41692989e-11, -3.08295720e-08,
                                       1.74400821e-05, -5.77168544e-03,  1.49577726e+00, -1.75503752e+04])
        
        coeff_cg_x_lox_cb = np.array([ 1.04192489e-36, -4.67523591e-32,  9.12288078e-28, -1.01596948e-23,
                                       7.12296436e-20, -3.27069624e-16,  9.91581397e-13, -1.95284081e-09,
                                       2.39493446e-06, -1.69276060e-03,  9.30797157e-01, -1.28073975e+04])
        
        cg_x_atf_cb = evaluate_poly_horner(atf_mass_cb, coeff_cg_x_atf_cb)
        cg_y_atf_cb = 0.0
        cg_z_atf_cb = 0.0
        
        cg_x_lox_cb = evaluate_poly_horner(lox_mass_cb, coeff_cg_x_lox_cb)
        cg_y_lox_cb = 0.0
        cg_z_lox_cb = 0.0
        
        cg_x_dry_cb = -15055.43	
        cg_y_dry_cb = -3.867
        cg_z_dry_cb = -10.667
        
        net_cg_x_numerator += cg_x_atf_cb * atf_mass_cb + cg_x_lox_cb * lox_mass_cb + cg_x_dry_cb * dry_mass_cb
        net_cg_y_numerator += cg_y_atf_cb * atf_mass_cb + cg_y_lox_cb * lox_mass_cb + cg_y_dry_cb * dry_mass_cb
        net_cg_z_numerator += cg_z_atf_cb * atf_mass_cb + cg_z_lox_cb * lox_mass_cb + cg_z_dry_cb * dry_mass_cb
        
        net_cg_denominator += atf_mass_cb + lox_mass_cb + dry_mass_cb

    if(is_PLF == True):
        cg_x_PLF = -1912.006
        cg_y_PLF = -4.076
        cg_z_PLF = -47.923
        net_cg_x_numerator += cg_x_PLF * PLF_mass
        net_cg_y_numerator += cg_y_PLF * PLF_mass
        net_cg_z_numerator += cg_z_PLF * PLF_mass
        net_cg_denominator += PLF_mass
        # print('PLF')

    if(is_upper_stage == True):
        # print('US')
        coeff_cg_x_atf_us = np.array([ 8.88768679e-23, -1.48768635e-19,  8.23154184e-17, -1.76692194e-15,
                                      -1.87291194e-11,  9.87696512e-09, -2.57926839e-06,  3.99402473e-04,
                                      -3.91279890e-02,  3.41940905e+00, -2.58559133e+03])
        
        coeff_cg_x_lox_us = np.array([ 3.33038267e-27, -1.41693365e-23,  1.96701365e-20,  1.12477580e-18,
                                      -3.61346985e-14,  5.00490539e-11, -3.58373791e-08,  1.55303142e-05,
                                      -4.18693416e-03,  1.13202696e+00, -3.95278506e+03])
        
        cg_x_atf_us = evaluate_poly_horner(atf_mass_us, coeff_cg_x_atf_us)
        cg_y_atf_us = 0.0
        cg_z_atf_us = 0.0
        
        cg_x_lox_us = evaluate_poly_horner(lox_mass_us, coeff_cg_x_lox_us)
        cg_y_lox_us = 0.0
        cg_z_lox_us = 0.0
        #Dry Stage
        cg_x_dry_us = -2769.803
        cg_y_dry_us = 5.986
        cg_z_dry_us = 26.821
        #Payload
        cg_x_payload = -1423.968
        cg_y_payload = -96.801
        cg_z_payload = -15.194

        net_cg_x_numerator += cg_x_atf_us * atf_mass_us + cg_x_lox_us * lox_mass_us + cg_x_dry_us * dry_mass_us + cg_x_payload * payload_mass
        net_cg_y_numerator += cg_y_atf_us * atf_mass_us + cg_y_lox_us * lox_mass_us + cg_y_dry_us * dry_mass_us + cg_y_payload * payload_mass
        net_cg_z_numerator += cg_z_atf_us * atf_mass_us + cg_z_lox_us * lox_mass_us + cg_z_dry_us * dry_mass_us + cg_z_payload * payload_mass
        net_cg_denominator += atf_mass_us + lox_mass_us + dry_mass_us + payload_mass

    net_cg_x = net_cg_x_numerator/net_cg_denominator
    net_cg_y = net_cg_y_numerator/net_cg_denominator
    net_cg_z = net_cg_z_numerator/net_cg_denominator
    
    return net_cg_x, net_cg_y, net_cg_z