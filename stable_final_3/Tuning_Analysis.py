# -*- coding: utf-8 -*-
# ==============================================================================
# File:         Tuning_Analysis.py
# Author:       Gemini
# Date:         September 5, 2025
# Description:  This module contains the logic for analyzing the results of a
#               calibration run. It performs linear regression on the collected
#               data to identify zero, span, and linearity errors, and generates
#               a formatted report with tuning suggestions.
#
# Version Update (NaN Handling):
#   - Ensured that NaN values (logged for out-of-range setpoints) are
#     correctly filtered out before performing linear regression.
# ==============================================================================

import numpy as np
import matplotlib.pyplot as plt

def generate_tuning_suggestions(data_storage, active_duts):
    """
    Analyzes calibration data and generates tuning suggestions for each DUT.

    Args:
        data_storage (dict): A dictionary containing the collected data from the run.
        active_duts (list): A list of dictionaries describing the active DUTs.

    Returns:
        str: A formatted string with tuning suggestions, or an empty string if
             no suggestions are needed.
        list: A list of file paths to the generated plot images.
    """
    suggestion_parts = ["--- Post-Calibration Tuning Analysis ---"]
    any_suggestions = False
    plot_filenames = []

    for dut in active_duts:
        ch, fs = dut['channel'], dut['fs']
        
        # Extract valid, non-NaN data points for the current DUT for analysis
        std_points = np.array([s for s, d in zip(data_storage['Standard_Pressure_Torr'], data_storage[f'Device_{ch+1}_Pressure_Torr']) if not np.isnan(d) and not np.isnan(s)])
        dut_points = np.array([d for s, d in zip(data_storage['Standard_Pressure_Torr'], data_storage[f'Device_{ch+1}_Pressure_Torr']) if not np.isnan(d) and not np.isnan(s)])
        
        if len(dut_points) < 3: continue

        # --- Perform Analysis ---
        slope, intercept = np.polyfit(std_points, dut_points, 1)
        zero_offset_is_sig = abs(intercept) > (fs * 0.001)
        span_error_is_sig = abs(1.0 - slope) > 0.005
        
        non_linearity_points = [dut_p - (slope * std_p + intercept) for std_p, dut_p in zip(std_points, dut_points)]
        linearity_is_sig = False
        midpoint_error_from_line = 0
        if non_linearity_points:
            max_non_linearity = max(np.abs(non_linearity_points))
            linearity_is_sig = max_non_linearity > (fs * 0.002)
            midpoint_error_from_line = non_linearity_points[np.abs(non_linearity_points).argmax()]

        # --- Generate Plot ---
        plot_filename = f"DUT_{ch+1}_tuning_curve.png"
        generate_curve_plot(std_points, dut_points, slope, intercept, fs, ch, plot_filename)
        plot_filenames.append(plot_filename)

        # --- Build Suggestion Text ---
        if not (zero_offset_is_sig or span_error_is_sig or linearity_is_sig):
            suggestion_parts.append(f"\n{'='*50}\n--- ✅ DUT {ch+1} ({fs} Torr FS): SUCCESS ---\n{'='*50}\nDevice is well-calibrated. No adjustments needed.")
            continue

        any_suggestions = True
        suggestion_parts.append(f"\n{'='*50}\n--- ⚠️ DUT {ch+1} ({fs} Torr FS): Adjustments Recommended ---\n{'='*50}")
        
        # Diagnosis
        suggestion_parts.append(f"\n[ DIAGNOSIS ]")
        suggestion_parts.append(f" • Equation: y = {slope:.4f}x + {intercept:+.4f}")
        if zero_offset_is_sig: suggestion_parts.append(f" • Zero Offset Error: {intercept:+.4f} Torr (Significant)")
        if span_error_is_sig: suggestion_parts.append(f" • Span (Gain) Error: Slope is {slope:.4f} ({'too high' if slope > 1 else 'too low'})")
        if linearity_is_sig: suggestion_parts.append(f" • Linearity Error: Mid-range response {'bows UP' if midpoint_error_from_line > 0 else 'bows DOWN'}")

        # Recommendations
        suggestion_parts.append("\n[ RECOMMENDED ACTIONS ]")
        if zero_offset_is_sig and abs(intercept) > (fs * 0.02):
             suggestion_parts.append("1. Coarse Zero Adjustment: The zero offset is large. Make a coarse adjustment first.")
        if zero_offset_is_sig:
            suggestion_parts.append(f"2. Zero Adjustment: Adjust the zero reading {'DOWN' if intercept > 0 else 'UP'}.")
        if span_error_is_sig:
            suggestion_parts.append(f"3. Span Adjustment: Adjust the span (at 100% FS) {'DOWN' if slope > 1 else 'UP'}.")
        if linearity_is_sig:
            suggestion_parts.append(f"4. Linearity Adjustment: Correct the {'upward \"smiling\"' if midpoint_error_from_line > 0 else 'downward \"frowning\"'} bow at 50% FS.")

    final_suggestion_text = "\n".join(suggestion_parts)
    return final_suggestion_text if any_suggestions else "", plot_filenames


def generate_curve_plot(std_points, dut_points, slope, intercept, fs, ch, filename):
    """Generates and saves a calibration curve plot for a single DUT."""
    fig, ax = plt.subplots(figsize=(8, 6))
    
    # Plot ideal line - using the DUT's FS for correct scaling
    ax.plot([0, fs], [0, fs], 'k--', label='Ideal Curve')
    
    # Plot measured data
    ax.plot(std_points, dut_points, 'bo-', label='Measured Data')
    
    # Highlight out-of-tolerance points
    for std, dut in zip(std_points, dut_points):
        if abs(std - dut) > (fs * 0.005):
            ax.plot(std, dut, 'rX', markersize=10, label='Out of Tolerance' if 'Out of Tolerance' not in [l.get_label() for l in ax.get_lines()] else "")

    ax.set_title(f'DUT {ch+1} ({fs} Torr FS) - Calibration Curve')
    ax.set_xlabel('Standard Pressure (Torr)')
    ax.set_ylabel(f'DUT {ch+1} Pressure (Torr)')
    ax.legend()
    ax.grid(True)
    
    # Set plot limits to the DUT's full scale for clarity
    ax.set_xlim(left=-0.05 * fs, right=fs * 1.05)
    ax.set_ylim(bottom=-0.05 * fs, top=fs * 1.05)
    
    fig.tight_layout()
    fig.savefig(filename)
    plt.close(fig)
