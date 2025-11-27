#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np

def calculate_odd_parity(bits_string):
    """Calculates the odd parity bit for a binary string."""
    ones_count = bits_string.count('1')
    return '0' if ones_count % 2 != 0 else '1'

def get_modified_miller_pulses(bits, et_u=128):
    """
    Generates a list of (duration, level) pulses for a given bit string,
    now including an initial sync pulse in the SOF.
    """
    pulses = []
    prev_bit = '1'
    is_first_bit = True

    for bit in bits:
        if bit == '1':
            pulses.extend([(et_u / 2, 1), (et_u / 4, 0), (et_u / 4, 1)])
            prev_bit = '1'
        elif bit == '0':
            # For the very first bit (SOF), force the '0 after 0' rule to create a sync pulse.
            if is_first_bit:
                pulses.extend([(et_u / 4, 0), (3 * et_u / 4, 1)])
                prev_bit = '0'
            elif prev_bit == '1':
                pulses.append((et_u, 1))
                prev_bit = '0'
            else: # prev_bit was '0'
                pulses.extend([(et_u / 4, 0), (3 * et_u / 4, 1)])
                prev_bit = '0'
        is_first_bit = False
    return pulses

def plot_signal_waveform(title, bits_to_encode, bit_labels, et_u=128):
    """
    Generic plotting function with precise rise-to-rise arrows and abbreviated timings.
    """
    pulses = get_modified_miller_pulses(bits_to_encode, et_u)
    
    # --- Generate Plot Coordinates ---
    x_coords, y_coords = [0], [pulses[0][1]]
    current_time = 0
    for duration, level in pulses:
        x_coords.extend([current_time, current_time + duration])
        y_coords.extend([level, level])
        current_time += duration
    
    # --- Create the Plot ---
    fig, ax = plt.subplots(figsize=(max(18, 1.2 * len(bits_to_encode)), 6.5))
    ax.plot(x_coords, y_coords, linewidth=2.5, color='tab:blue')

    # --- Annotations and Styling ---
    ax.set_title(title, fontsize=16, pad=30)
    ax.set_xlabel("Time (t)", fontsize=12)
    ax.set_ylabel("Signal Level", fontsize=12)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["LOW", "HIGH"])
    ax.set_ylim(-0.4, 1.4)
    ax.set_xlim(-et_u * 0.1, len(bits_to_encode) * et_u * 1.01)
    ax.set_xticks(np.arange(0, len(bits_to_encode) * et_u + 1, et_u))

    # Annotate each pulse duration with abbreviated "t"
    time_cursor = 0
    for duration, level in pulses:
        text_x = time_cursor + duration / 2
        text_y = level - 0.08 if level > 0.5 else level + 0.08
        vertical_align = 'top' if level > 0.5 else 'bottom'
        if duration > 0:
            ax.text(text_x, text_y, f"{int(duration)}t", ha='center', va=vertical_align, fontsize=8, color='black')
        time_cursor += duration

    # Annotate each bit period
    for i, label in enumerate(bit_labels):
        bit_start_time = i * et_u
        bit_value = bits_to_encode[i]
        full_label = f"{label}: {bit_value}"
        ax.text(bit_start_time + et_u/2, 1.2, full_label, ha='center', fontsize=9,
                bbox=dict(boxstyle="round,pad=0.3", fc='aliceblue', ec='grey', lw=0.5))
        if i > 0:
            ax.axvline(x=bit_start_time, color='green', linestyle='--', linewidth=1.5, alpha=0.8)
        for j in range(1, 4):
            ax.axvline(x=bit_start_time + j * et_u / 4, color='grey', linestyle=':', linewidth=0.7, alpha=0.6)

    # --- NEW: Add precise Rise-to-Rise Timing Arrows ---
    rising_edge_times = []
    current_time = 0
    # Find all rising edges
    for i in range(len(pulses) - 1):
        if pulses[i][1] == 0 and pulses[i+1][1] == 1:
             # A rising edge occurs at the transition point
             rising_edge_times.append(current_time + pulses[i][0])
        current_time += pulses[i][0]

    # Calculate deltas and plot arrows
    y_pos = -0.25
    if len(rising_edge_times) > 1:
        for i in range(1, len(rising_edge_times)):
            t1, t2 = rising_edge_times[i-1], rising_edge_times[i]
            delta = t2 - t1
            # Draw the arrow spanning the full duration
            ax.annotate("", xy=(t1, y_pos), xytext=(t2, y_pos),
                        arrowprops=dict(arrowstyle="<|-|>,head_width=0.4,head_length=0.8",
                                        color='darkmagenta', shrinkA=0, shrinkB=0, lw=1))
            # Add the text in the middle with a white background to break the line
            ax.text((t1 + t2) / 2, y_pos, f"{int(delta)}t", ha='center', va='center',
                    color='darkmagenta', fontsize=9,
                    bbox=dict(boxstyle="square,pad=0.0", fc='white', ec='none'))

    ax.grid(True, which='major', axis='x', linestyle=':', color='gray', alpha=0.7)
    plt.tight_layout()
    plt.show()

def plot_nfc_command(command_name, command_bytes, et_u=128):
    """
    Main function to generate and plot an NFC command signal.
    """
    short_frame_cmds = [0x26, 0x52]
    
    if len(command_bytes) == 1 and command_bytes[0] in short_frame_cmds:
        byte_val = command_bytes[0]
        data_bits = f'{byte_val:07b}'[::-1]
        bit_labels = ["SOF"] + [f"D{i}" for i in range(7)] + ["EOF"]
        title_suffix = "(Short Frame)"
    else:
        data_bits = ""
        bit_labels = ["SOF"]
        for i, byte_val in enumerate(command_bytes):
            byte_bits = f'{byte_val:08b}'
            parity = calculate_odd_parity(byte_bits)
            data_bits += byte_bits[::-1] + parity
            bit_labels.extend([f"B{i}D{j}" for j in range(8)])
            bit_labels.append(f"B{i}P")
        bit_labels.append("EOF")
        title_suffix = "(Standard Frame with Parity)"
        
    bits_to_encode = '0' + data_bits + '0'
    
    hex_str = ' '.join([f'0x{b:02X}' for b in command_bytes])
    title = f"'{command_name}' [{hex_str}] {title_suffix}"
    plot_signal_waveform(title, bits_to_encode, bit_labels, et_u)

# --- EXAMPLES ---
plot_nfc_command("REQA", [0x26])
plot_nfc_command("WUPA", [0x52])
plot_nfc_command("HALT", [0x50, 0x00, 0x57, 0xcd])
plot_nfc_command("Anticollision CL1", [0x93, 0x20])
plot_nfc_command("Anticollision CL2", [0x95, 0x20])
plot_nfc_command("Anticollision CL3", [0x97, 0x20])
plot_nfc_command("READ 0x00", [0x30, 0x00, 0x02, 0xa8])
