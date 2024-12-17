"""
Code Title: Moment of Inertia Estimation Tool
Code Version: V2.0
Developed By: Ajaya Kumar Patel
Release Date: 08-11-2024
CAD Data Version: V3.2

Description:
MIEstimation Tool, a software designed for calculating the moment of inertia across the entire flight duration. 
The tool determines the MI as a function of mass, ensuring accurate tracking of MI change as propellant is consumed.

Features:
- Horner’s method is used to evaluate polynomial in linear time

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

coeffs_path = 'C://Users//akpat//Desktop//Agnikul//Agniban01//17-12//CG MI Model V3.2//MI Coefficients//'

def mi_estimator(b1_mass, b2_mass, cb_mass, s2_mass, cg_nose, is_b1 = True, is_b2 = True, core_booster = True, is_PLF = True, second_stage = True):
    """
    mass : array with current mass of vehicle [mass booster-1, mass booster-2, mass core boster, mass stage-2]
    cg_nose: CG in nose frame [x, y, z]
    Python: Select Interpreter
    
    """
    m_b1_atf, m_b1_lox, m_b1_dry = b1_mass
    m_b2_atf, m_b2_lox, m_b2_dry = b2_mass
    m_cb_atf, m_cb_lox, m_cb_dry = cb_mass
    m_s2_atf, m_s2_lox, m_s2_dry, m_payload, m_PLF = s2_mass

    m_tot = 0
    x,y,z = cg_nose
    I_nose = np.zeros((3,3))
    if(is_b1 == True):
        # print('PYB')
        m_tot += m_b1_atf + m_b1_lox + m_b1_dry
        I_SB1_dry = np.array([[5795665140.8, 56467332726.272, -300382526.357],
                              [56467332726.272, 679729943281.927 , 27919050.456], 
                              [-300382526.357, 27919050.456, 684825076152.85]])

        SB1_atf_coeff = pd.read_csv(coeffs_path+'PYB_ATF_coefficient.csv')
        SB1_lox_coeff = pd.read_csv(coeffs_path+'PYB_LOX_coefficient.csv')
        #booster 1
        Ixx_SB1_atf = evaluate_poly_horner(m_b1_atf, SB1_atf_coeff['Ixx'].values)
        Iyx_SB1_atf = evaluate_poly_horner(m_b1_atf, SB1_atf_coeff['Iyx'].values)
        Iyy_SB1_atf = evaluate_poly_horner(m_b1_atf, SB1_atf_coeff['Iyy'].values)
        Izx_SB1_atf = evaluate_poly_horner(m_b1_atf, SB1_atf_coeff['Izx'].values)
        Izy_SB1_atf = evaluate_poly_horner(m_b1_atf, SB1_atf_coeff['Izy'].values)
        Izz_SB1_atf = evaluate_poly_horner(m_b1_atf, SB1_atf_coeff['Izz'].values)
        
        Ixx_SB1_lox = evaluate_poly_horner(m_b1_lox, SB1_lox_coeff['Ixx'].values)
        Iyx_SB1_lox = evaluate_poly_horner(m_b1_lox, SB1_lox_coeff['Iyx'].values)
        Iyy_SB1_lox = evaluate_poly_horner(m_b1_lox, SB1_lox_coeff['Iyy'].values)
        Izx_SB1_lox = evaluate_poly_horner(m_b1_lox, SB1_lox_coeff['Izx'].values)
        Izy_SB1_lox = evaluate_poly_horner(m_b1_lox, SB1_lox_coeff['Izy'].values)
        Izz_SB1_lox = evaluate_poly_horner(m_b1_lox, SB1_lox_coeff['Izz'].values)

        I_SB1_atf = np.array([[Ixx_SB1_atf, Iyx_SB1_atf, Izx_SB1_atf],
                              [Iyx_SB1_atf, Iyy_SB1_atf, Izy_SB1_atf], 
                              [Izx_SB1_atf, Izy_SB1_atf, Izz_SB1_atf]])

        I_SB1_lox = np.array([[Ixx_SB1_lox, Iyx_SB1_lox, Izx_SB1_lox], 
                              [Iyx_SB1_lox, Iyy_SB1_lox, Izy_SB1_lox], 
                              [Izx_SB1_lox, Izy_SB1_lox, Izz_SB1_lox]])
        I_nose += I_SB1_dry + I_SB1_atf + I_SB1_lox
        
    if(is_b2 == True):
        # print('MYB')
        m_tot += m_b2_atf + m_b2_lox + m_b2_dry
        I_SB2_dry = np.array([[5795665140.841, -56470000000, 300382526.359],
                              [-56470000000, 679729943281.886, 27919050.456],
                              [300382526.359, 27919050.456, 684825076152.851]])

        SB2_atf_coeff = pd.read_csv(coeffs_path+'MYB_ATF_coefficient.csv')
        SB2_lox_coeff = pd.read_csv(coeffs_path+'MYB_LOX_coefficient.csv')
        
        #booster 2
        Ixx_SB2_atf = evaluate_poly_horner(m_b2_atf, SB2_atf_coeff['Ixx'].values)
        Iyx_SB2_atf = evaluate_poly_horner(m_b2_atf, SB2_atf_coeff['Iyx'].values)
        Iyy_SB2_atf = evaluate_poly_horner(m_b2_atf, SB2_atf_coeff['Iyy'].values)
        Izx_SB2_atf = evaluate_poly_horner(m_b2_atf, SB2_atf_coeff['Izx'].values)
        Izy_SB2_atf = evaluate_poly_horner(m_b2_atf, SB2_atf_coeff['Izy'].values)
        Izz_SB2_atf = evaluate_poly_horner(m_b2_atf, SB2_atf_coeff['Izz'].values)

        Ixx_SB2_lox = evaluate_poly_horner(m_b2_lox, SB2_lox_coeff['Ixx'].values)
        Iyx_SB2_lox = evaluate_poly_horner(m_b2_lox, SB2_lox_coeff['Iyx'].values)
        Iyy_SB2_lox = evaluate_poly_horner(m_b2_lox, SB2_lox_coeff['Iyy'].values)
        Izx_SB2_lox = evaluate_poly_horner(m_b2_lox, SB2_lox_coeff['Izx'].values)
        Izy_SB2_lox = evaluate_poly_horner(m_b2_lox, SB2_lox_coeff['Izy'].values)
        Izz_SB2_lox = evaluate_poly_horner(m_b2_lox, SB2_lox_coeff['Izz'].values)
        
        I_SB2_atf = np.array([[Ixx_SB2_atf, Iyx_SB2_atf, Izx_SB2_atf], 
                              [Iyx_SB2_atf, Iyy_SB2_atf, Izy_SB2_atf], 
                              [Izx_SB2_atf, Izy_SB2_atf, Izz_SB2_atf]])

        I_SB2_lox = np.array([[Ixx_SB2_lox, Iyx_SB2_lox, Izx_SB2_lox], 
                              [Iyx_SB2_lox, Iyy_SB2_lox, Izy_SB2_lox], 
                              [Izx_SB2_lox, Izy_SB2_lox, Izz_SB2_lox]])
        
        I_nose += I_SB2_dry + I_SB2_atf + I_SB2_lox
    if (core_booster == True):
        # print('CB')
        m_tot += m_cb_atf + m_cb_lox + m_cb_dry
        I_core_dry = np.array([[791339080.314, -133440923.739, -411604599.838],
                               [-133440923.739, 622232104139.762, -386164.932],
                               [-411604599.838, -386164.932, 622225892090.909]])
        
        CB_atf_coeff = pd.read_csv(coeffs_path+'CB_ATF_coefficient.csv')
        CB_lox_coeff = pd.read_csv(coeffs_path+'CB_LOX_coefficient.csv')
        
        Ixx_core_atf = evaluate_poly_horner(m_cb_atf, CB_atf_coeff['Ixx'].values)
        Iyx_core_atf = evaluate_poly_horner(m_cb_atf, CB_atf_coeff['Iyx'].values)
        Iyy_core_atf = evaluate_poly_horner(m_cb_atf, CB_atf_coeff['Iyy'].values)
        Izx_core_atf = evaluate_poly_horner(m_cb_atf, CB_atf_coeff['Izx'].values)
        Izy_core_atf = evaluate_poly_horner(m_cb_atf, CB_atf_coeff['Izy'].values)
        Izz_core_atf = evaluate_poly_horner(m_cb_atf, CB_atf_coeff['Izz'].values)

        Ixx_core_lox = evaluate_poly_horner(m_cb_lox, CB_lox_coeff['Ixx'].values)
        Iyx_core_lox = evaluate_poly_horner(m_cb_lox, CB_lox_coeff['Iyx'].values)
        Iyy_core_lox = evaluate_poly_horner(m_cb_lox, CB_lox_coeff['Iyy'].values)
        Izx_core_lox = evaluate_poly_horner(m_cb_lox, CB_lox_coeff['Izx'].values)
        Izy_core_lox = evaluate_poly_horner(m_cb_lox, CB_lox_coeff['Izy'].values)
        Izz_core_lox = evaluate_poly_horner(m_cb_lox, CB_lox_coeff['Izz'].values)
        
        I_core_atf = np.array([[Ixx_core_atf, Iyx_core_atf, Izx_core_atf], 
                               [Iyx_core_atf, Iyy_core_atf, Izy_core_atf], 
                               [Izx_core_atf, Izy_core_atf, Izz_core_atf]])

        I_core_lox = np.array([[Ixx_core_lox, Iyx_core_lox, Izx_core_lox], 
                               [Iyx_core_lox, Iyy_core_lox, Izy_core_lox], 
                               [Izx_core_lox, Izy_core_lox, Izz_core_lox]])

        I_nose += I_core_dry + I_core_atf + I_core_lox    


    if(is_PLF == True):
        # print('PLF')
        m_tot += m_PLF
        I_PLF = np.array([[67274042.831, -1112423.045, -14218063.664],
                          [-1112423.045, 722763061.19, 149122.8],
                          [-14218063.664, 149122.8, 726177250.87]])
        I_nose += I_PLF

    if(second_stage == True):
        # print('US')
        m_tot += m_s2_atf + m_s2_lox + m_s2_dry + m_payload
        
        I_s2_dry = np.array([[87255574.872, 3803998.094 , 19330686.536],
                             [3803998.094, 3727996516.662, 643673.629],
                             [19330686.536, 643673.629, 3725862603.407]])
        
        I_payload = np.array([[56339.831, -689208.066, -108178.2],
                              [-689208.066, 10147912.613, -7353.932],
                              [-108178.2, -7353.932, 10193610.563]])
        
        s2_atf_coeff = pd.read_csv(coeffs_path+'US_ATF_coefficient.csv')
        s2_lox_coeff = pd.read_csv(coeffs_path+'US_LOX_coefficient.csv')
        
        Ixx_s2_atf = evaluate_poly_horner(m_s2_atf, s2_atf_coeff['Ixx'].values)
        Iyx_s2_atf = evaluate_poly_horner(m_s2_atf, s2_atf_coeff['Iyx'].values)
        Iyy_s2_atf = evaluate_poly_horner(m_s2_atf, s2_atf_coeff['Iyy'].values)
        Izx_s2_atf = evaluate_poly_horner(m_s2_atf, s2_atf_coeff['Izx'].values)
        Izy_s2_atf = evaluate_poly_horner(m_s2_atf, s2_atf_coeff['Izy'].values)
        Izz_s2_atf = evaluate_poly_horner(m_s2_atf, s2_atf_coeff['Izz'].values)

        Ixx_s2_lox = evaluate_poly_horner(m_s2_lox, s2_lox_coeff['Ixx'].values)
        Iyx_s2_lox = evaluate_poly_horner(m_s2_lox, s2_lox_coeff['Iyx'].values)
        Iyy_s2_lox = evaluate_poly_horner(m_s2_lox, s2_lox_coeff['Iyy'].values)
        Izx_s2_lox = evaluate_poly_horner(m_s2_lox, s2_lox_coeff['Izx'].values)
        Izy_s2_lox = evaluate_poly_horner(m_s2_lox, s2_lox_coeff['Izy'].values)
        Izz_s2_lox = evaluate_poly_horner(m_s2_lox, s2_lox_coeff['Izz'].values)
        
        I_s2_atf = np.array([[Ixx_s2_atf, Iyx_s2_atf, Izx_s2_atf], 
                             [Iyx_s2_atf, Iyy_s2_atf, Izy_s2_atf], 
                             [Izx_s2_atf, Izy_s2_atf, Izz_s2_atf]])

        I_s2_lox = np.array([[Ixx_s2_lox, Iyx_s2_lox, Izx_s2_lox], 
                             [Iyx_s2_lox, Iyy_s2_lox, Izy_s2_lox], 
                             [Izx_s2_lox, Izy_s2_lox, Izz_s2_lox]])
        
        I_nose += I_s2_dry + I_s2_atf + I_s2_lox + I_payload

    R_sq = np.array([[y**2 + z**2, -x*y, -x*z], 
                     [-y*x, x**2+z**2, -y*z], 
                     [-z*x, -z*y, x**2+y**2]])

    I_cg = I_nose - m_tot*R_sq
    # print(I_nose)
    Ixx, Iyy, Izz, Iyx, Izx, Izy = I_cg[0][0], I_cg[1][1], I_cg[2][2], I_cg[0][1], I_cg[0][2], I_cg[1][2]
    return Ixx, Iyy, Izz, Iyx, Izx, Izy