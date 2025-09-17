# final/Tuning_Analysis.py
import numpy as np
import matplotlib.pyplot as plt
import os # <-- Import the os module

def generate_tuning_suggestions(data_storage, active_duts):
    """
    Analyzes calibration data and generates tuning suggestions for each DUT.
    """
    suggestion_reports = {}
    plot_filenames = {}
    dut_pass_status = {}
    
    # --- NEW: Create the Analysis directory if it doesn't exist ---
    output_dir = "Analysis"
    os.makedirs(output_dir, exist_ok=True)

    for dut in active_duts:
        ch, fs = dut['channel'], dut['fs']
        suggestion_parts = []
        
        std_points = np.array([s for s, d in zip(data_storage['Standard_Pressure_Torr'], data_storage[f'Device_{ch+1}_Pressure_Torr']) if not np.isnan(d) and not np.isnan(s)])
        dut_points = np.array([d for s, d in zip(data_storage['Standard_Pressure_Torr'], data_storage[f'Device_{ch+1}_Pressure_Torr']) if not np.isnan(d) and not np.isnan(s)])
        
        if len(dut_points) < 3:
            dut_pass_status[ch] = False
            suggestion_reports[ch] = f"\n--- ⚠️ DUT {ch+1} ({fs} Torr FS): INSUFFICIENT DATA ---\nNot enough valid data points to perform analysis."
            plot_filenames[ch] = None
            continue

        slope, intercept = np.polyfit(std_points, dut_points, 1)
        zero_offset_is_sig = abs(intercept) > (fs * 0.001)
        span_error_is_sig = abs(1.0 - slope) > 0.005
        
        non_linearity_points = [dut_p - (slope * std_p + intercept) for std_p, dut_p in zip(std_points, dut_points)]
        linearity_is_sig = False
        midpoint_error_from_line = 0
        if non_linearity_points:
            max_non_linearity = max(np.abs(non_linearity_points))
            linearity_is_sig = max_non_linearity > (fs * 0.002)
            if linearity_is_sig:
                 midpoint_error_from_line = non_linearity_points[np.abs(non_linearity_points).argmax()]

        # --- MODIFIED: Filename now includes the "Analysis" directory ---
        plot_filename = os.path.join(output_dir, f"DUT_{ch+1}_tuning_curve.png")
        generate_curve_plot(std_points, dut_points, slope, intercept, fs, ch, plot_filename)
        plot_filenames[ch] = plot_filename

        is_pass = not (zero_offset_is_sig or span_error_is_sig or linearity_is_sig)
        dut_pass_status[ch] = is_pass

        if is_pass:
            suggestion_parts.append(f"--- ✅ DUT {ch+1} ({fs} Torr FS): SUCCESS ---")
            suggestion_parts.append("Device is well-calibrated and within tolerance.")
            suggestion_parts.append(f" • Equation: y = {slope:.4f}x + {intercept:+.4f}")

        else:
            suggestion_parts.append(f"--- ⚠️ DUT {ch+1} ({fs} Torr FS): Adjustments Recommended ---")
            suggestion_parts.append(f"\n[ DIAGNOSIS ]")
            suggestion_parts.append(f" • Equation: y = {slope:.4f}x + {intercept:+.4f}")
            if zero_offset_is_sig: suggestion_parts.append(f" • Zero Offset Error: {intercept:+.4f} Torr (Significant)")
            if span_error_is_sig: suggestion_parts.append(f" • Span (Gain) Error: Slope is {slope:.4f} ({'too high' if slope > 1 else 'too low'})")
            if linearity_is_sig: suggestion_parts.append(f" • Linearity Error: Mid-range response {'bows UP' if midpoint_error_from_line > 0 else 'bows DOWN'}")

            suggestion_parts.append("\n[ RECOMMENDED ACTIONS ]")
            if zero_offset_is_sig and abs(intercept) > (fs * 0.02):
                 suggestion_parts.append("1. Coarse Zero Adjustment: The zero offset is large. Make a coarse adjustment first.")
            if zero_offset_is_sig:
                suggestion_parts.append(f"2. Zero Adjustment: Adjust the zero reading {'DOWN' if intercept > 0 else 'UP'}.")
            if span_error_is_sig:
                suggestion_parts.append(f"3. Span Adjustment: Adjust the span (at 100% FS) {'DOWN' if slope > 1 else 'UP'}.")
            if linearity_is_sig:
                suggestion_parts.append(f"4. Linearity Adjustment: Correct the {'upward \"smiling\"' if midpoint_error_from_line > 0 else 'downward \"frowning\"'} bow at 50% FS.")

        suggestion_reports[ch] = "\n".join(suggestion_parts)

    return suggestion_reports, plot_filenames, dut_pass_status


def generate_curve_plot(std_points, dut_points, slope, intercept, fs, ch, filename):
    """Generates and saves a calibration curve plot for a single DUT."""
    fig, ax = plt.subplots(figsize=(8, 6))
    
    ax.plot([0, fs], [0, fs], 'k--', label='Ideal Curve')
    ax.plot(std_points, dut_points, 'bo-', label='Measured Data')
    
    for std, dut in zip(std_points, dut_points):
        if abs(std - dut) > (fs * 0.005):
            ax.plot(std, dut, 'rX', markersize=10, label='Out of Tolerance' if 'Out of Tolerance' not in [l.get_label() for l in ax.get_lines()] else "")

    ax.set_title(f'DUT {ch+1} ({fs} Torr FS) - Calibration Curve')
    ax.set_xlabel('Standard Pressure (Torr)')
    ax.set_ylabel(f'DUT {ch+1} Pressure (Torr)')
    ax.legend()
    ax.grid(True)
    
    ax.set_xlim(left=-0.05 * fs, right=fs * 1.05)
    ax.set_ylim(bottom=-0.05 * fs, top=fs * 1.05)
    
    fig.tight_layout()
    fig.savefig(filename)
    plt.close(fig)