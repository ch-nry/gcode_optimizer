import re
import math
import sys
import os
from typing import List, Tuple, Dict

# =============================================================================
# CONFIGURATION PARAMETERS
# =============================================================================
SAFE_Z = 5.0              # Safe height for Z moves
RAPID_FEEDRATE = 1500.0   # Feedrate for rapid moves G0
PLUNGE_FEEDRATE = 300.0   # Feedrate for Z plunge moves
DEFAULT_FEEDRATE = 500.0  # Default feedrate if none specified in G-code
# =============================================================================

class GCodeOptimizer:
    def __init__(self, filename: str, verbose: bool = False):
        self.filename = filename
        self.gcode_lines = []
        self.segments = []
        self.verbose = verbose
        
    def log(self, message: str):
        """Print debug message if verbose mode is enabled."""
        if self.verbose:
            print(f"[DEBUG] {message}")
        
    def read_gcode(self) -> List[str]:
        """Read G-code file and return lines."""
        with open(self.filename, 'r') as f:
            self.gcode_lines = f.readlines()
        self.log(f"Read {len(self.gcode_lines)} lines from file")
        return self.gcode_lines
    
    def parse_coordinates(self, line: str) -> Dict[str, float]:
        """Extract X, Y, Z, F coordinates from a G-code line."""
        coords = {}
        x_match = re.search(r'X([-+]?\d*\.?\d+)', line, re.IGNORECASE)
        y_match = re.search(r'Y([-+]?\d*\.?\d+)', line, re.IGNORECASE)
        z_match = re.search(r'Z([-+]?\d*\.?\d+)', line, re.IGNORECASE)
        f_match = re.search(r'F([-+]?\d*\.?\d+)', line, re.IGNORECASE)
        
        if x_match:
            coords['X'] = float(x_match.group(1))
        if y_match:
            coords['Y'] = float(y_match.group(1))
        if z_match:
            coords['Z'] = float(z_match.group(1))
        if f_match:
            coords['F'] = float(f_match.group(1))
            
        return coords
    
    def is_movement_command(self, line: str) -> bool:
        """Check if line is a movement command (G0 or G1)."""
        return bool(re.match(r'^\s*G[01]\s', line, re.IGNORECASE))
    
    def get_all_z_levels(self) -> List[float]:
        """Extract all unique Z levels from the file, sorted from highest to lowest."""
        z_values = set()
        current_z = None
        
        for line in self.gcode_lines:
            if self.is_movement_command(line):
                coords = self.parse_coordinates(line)
                if 'Z' in coords:
                    current_z = coords['Z']
                    if current_z <= 0:
                        z_values.add(current_z)
        
        sorted_z = sorted(z_values, reverse=True)
        self.log(f"Found {len(sorted_z)} Z levels: {sorted_z}")
        return sorted_z
    
    def extract_segments_at_z(self, target_z: float, tolerance: float = 0.001) -> List[Tuple[int, str, Dict, Dict, float]]:
        """
        Extract movement segments (start -> end) at specific Z level.
        Returns list of (line_number, original_line, start_coords, end_coords, feedrate).
        """
        segments = []
        current_z = None
        current_x = None
        current_y = None
        current_f = None
        z_positive_count = 0
        z_target_count = 0
        movement_count = 0
        z_none_count = 0
        
        self.log(f"Looking for segments at Z = {target_z} (tolerance = {tolerance})")
        
        for idx, line in enumerate(self.gcode_lines):
            if self.is_movement_command(line):
                movement_count += 1
                coords = self.parse_coordinates(line)
                
                start_x = current_x
                start_y = current_y
                start_z = current_z
                
                if 'X' in coords:
                    current_x = coords['X']
                if 'Y' in coords:
                    current_y = coords['Y']
                if 'Z' in coords:
                    current_z = coords['Z']
                    if self.verbose:
                        self.log(f"Line {idx}: Z changed to {current_z}")
                if 'F' in coords:
                    current_f = coords['F']
                    if self.verbose:
                        self.log(f"Line {idx}: F changed to {current_f}")
                
                if current_z is None:
                    z_none_count += 1
                    if self.verbose:
                        self.log(f"Line {idx}: Skipping (Z never set yet)")
                    continue
                
                if current_z > 0:
                    z_positive_count += 1
                    if self.verbose:
                        self.log(f"Line {idx}: Skipping (Z={current_z} > 0)")
                    continue
                
                if abs(current_z - target_z) <= tolerance:
                    if ('X' in coords or 'Y' in coords) and start_x is not None and start_y is not None:
                        start_coords = {'X': start_x, 'Y': start_y, 'Z': start_z if start_z else current_z}
                        end_coords = {'X': current_x, 'Y': current_y, 'Z': current_z}
                        
                        # Use current feedrate, or default from config if never set
                        feedrate = current_f if current_f is not None else DEFAULT_FEEDRATE
                        
                        segments.append((idx, line.strip(), start_coords, end_coords, feedrate))
                        z_target_count += 1
                        if self.verbose:
                            z_in_line = "Z in line" if 'Z' in coords else "Z inherited"
                            f_in_line = "F in line" if 'F' in coords else "F inherited"
                            self.log(f"Line {idx}: SEGMENT! ({z_in_line}, {f_in_line})")
                            self.log(f"         Start: ({start_x:.3f}, {start_y:.3f})")
                            self.log(f"         End:   ({current_x:.3f}, {current_y:.3f})")
                            self.log(f"         F: {feedrate}")
                else:
                    if self.verbose:
                        self.log(f"Line {idx}: Z={current_z} not matching target {target_z}")
        
        if not self.verbose:
            print(f"  Movements found: {movement_count}, at target Z: {z_target_count}")
        
        return segments
    
    def calculate_segment_length(self, start: Dict, end: Dict) -> float:
        """Calculate length of a segment."""
        x1 = start.get('X', 0)
        y1 = start.get('Y', 0)
        x2 = end.get('X', 0)
        y2 = end.get('Y', 0)
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def calculate_distance_between_segments(self, seg1_end: Dict, seg2_start: Dict) -> float:
        """Calculate distance between end of segment 1 and start of segment 2."""
        x1 = seg1_end.get('X', 0)
        y1 = seg1_end.get('Y', 0)
        x2 = seg2_start.get('X', 0)
        y2 = seg2_start.get('Y', 0)
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def nearest_neighbor_sort(self, segments: List[Tuple[int, str, Dict, Dict, float]], allow_reverse: bool = False) -> List[Tuple[int, str, Dict, Dict, bool, float]]:
        """
        Sort segments using nearest neighbor algorithm.
        Returns list with added boolean flag: (line_num, line, start, end, is_reversed, feedrate)
        """
        if not segments:
            return []
        
        self.log(f"Optimizing {len(segments)} segments, allow_reverse={allow_reverse}")
        
        # Unpack feedrate from input segments
        first_seg = segments[0]
        optimized = [(first_seg[0], first_seg[1], first_seg[2], first_seg[3], False, first_seg[4])]
        remaining = segments[1:]
        reverse_count = 0
        
        while remaining:
            last_seg = optimized[-1]
            if last_seg[4]:  # is_reversed
                current_position = last_seg[2]  # start
            else:
                current_position = last_seg[3]  # end
            
            min_dist = float('inf')
            nearest_idx = 0
            nearest_reversed = False
            
            for idx, (_, _, start, end, feedrate) in enumerate(remaining):
                dist_to_start = self.calculate_distance_between_segments(current_position, start)
                
                if allow_reverse:
                    dist_to_end = self.calculate_distance_between_segments(current_position, end)
                    
                    if dist_to_end < dist_to_start and dist_to_end < min_dist:
                        min_dist = dist_to_end
                        nearest_idx = idx
                        nearest_reversed = True
                    elif dist_to_start < min_dist:
                        min_dist = dist_to_start
                        nearest_idx = idx
                        nearest_reversed = False
                else:
                    if dist_to_start < min_dist:
                        min_dist = dist_to_start
                        nearest_idx = idx
                        nearest_reversed = False
            
            seg = remaining.pop(nearest_idx)
            optimized.append((seg[0], seg[1], seg[2], seg[3], nearest_reversed, seg[4]))
            if nearest_reversed:
                reverse_count += 1
        
        if allow_reverse:
            print(f"  Segments reversed: {reverse_count}/{len(segments)}")
        
        return optimized
    
    def optimize(self, target_z: float = None, output_filename: str = None, allow_reverse: bool = False, all_layers: bool = True, allow_comments: bool = False, force_overwrite: bool = False):
        """Main optimization function."""
        # Check if output file exists
        if os.path.exists(output_filename) and not force_overwrite:
            response = input(f"⚠️  File '{output_filename}' already exists. Overwrite? (y/n): ").strip().lower()
            if response not in ['y', 'yes', 'o', 'oui']:
                print("❌ Operation cancelled.")
                sys.exit(0)
        
        print(f"Reading G-code from: {self.filename}")
        self.read_gcode()
        
        if all_layers:
            print("\n🔍 Detecting all Z layers...")
            z_levels = self.get_all_z_levels()
            if not z_levels:
                print("❌ No negative Z levels found!")
                return
            print(f"✓ Found {len(z_levels)} layers: {z_levels}")
        else:
            z_levels = [target_z]
            print(f"\nOptimizing single layer at Z = {target_z}")
        
        all_optimized_segments = []
        total_segments = 0
        total_reversed = 0
        
        for layer_idx, z in enumerate(z_levels):
            print(f"\n{'='*60}")
            print(f"Layer {layer_idx + 1}/{len(z_levels)}: Z = {z}")
            print(f"{'='*60}")
            
            segments = self.extract_segments_at_z(z)
            
            if not segments:
                print(f"⚠️  No segments found, skipping")
                continue
            
            print(f"✓ Found {len(segments)} segments")
            
            optimized_segments = self.nearest_neighbor_sort(segments, allow_reverse)
            
            reversed_count = sum(1 for seg in optimized_segments if seg[4])
            total_reversed += reversed_count
            
            cutting_length = sum(self.calculate_segment_length(start, end) for _, _, start, end, _ in segments)
            
            original_travel = sum(
                self.calculate_distance_between_segments(segments[i][3], segments[i+1][2])
                for i in range(len(segments)-1)
            ) if len(segments) > 1 else 0
            
            optimized_travel = 0
            for i in range(len(optimized_segments)-1):
                current_seg = optimized_segments[i]
                next_seg = optimized_segments[i+1]
                
                current_end = current_seg[2] if current_seg[4] else current_seg[3]
                next_start = next_seg[3] if next_seg[4] else next_seg[2]
                
                optimized_travel += self.calculate_distance_between_segments(current_end, next_start)
            
            original_total = cutting_length + original_travel
            optimized_total = cutting_length + optimized_travel
            
            print(f"  Cutting: {cutting_length:.2f}mm")
            print(f"  Rapid travel: {original_travel:.2f}mm → {optimized_travel:.2f}mm", end="")
            if original_travel > 0:
                travel_saving = (original_travel - optimized_travel) / original_travel * 100
                print(f" (saved {travel_saving:.1f}%)")
            else:
                print()
            print(f"  Total distance: {original_total:.2f}mm → {optimized_total:.2f}mm", end="")
            if original_total > 0:
                total_saving = (original_total - optimized_total) / original_total * 100
                print(f" (saved {total_saving:.1f}%)")
            else:
                print()
            
            all_optimized_segments.append((z, optimized_segments))
            total_segments += len(segments)
        
        if not all_optimized_segments:
            print("\n❌ No segments found at any Z level!")
            return
        
        print(f"\n{'='*60}")
        print(f"TOTAL: {len(all_optimized_segments)} layers, {total_segments} segments")
        if allow_reverse:
            print(f"Total reversed: {total_reversed}")
        print(f"{'='*60}")
        
        self.write_gcode(all_optimized_segments, output_filename, allow_comments)
        print(f"\n✓ Optimized G-code written to: {output_filename}")
    
    def write_gcode(self, layers: List[Tuple[float, List]], output_filename: str, allow_comments: bool = False):
        """Write optimized G-code for all layers."""
        total_segments = sum(len(segments) for _, segments in layers)
        
        print(f"\nWriting {total_segments} segments across {len(layers)} layers...")
        print(f"  Safe Z: {SAFE_Z}")
        print(f"  Rapid feedrate: F{RAPID_FEEDRATE:.0f}")
        print(f"  Plunge feedrate: F{PLUNGE_FEEDRATE:.0f}")
        
        with open(output_filename, 'w') as f:
            if allow_comments:
                f.write("; Optimized G-code - Multi-layer\n")
                f.write(f"; Original file: {self.filename}\n")
                f.write(f"; Total layers: {len(layers)}\n")
                f.write(f"; Total segments: {total_segments}\n")
                f.write(f"; Safe Z: {SAFE_Z}\n")
                f.write(f"; Rapid feedrate: F{RAPID_FEEDRATE:.0f}\n")
                f.write(f"; Plunge feedrate: F{PLUNGE_FEEDRATE:.0f}\n")
                f.write(f"; Layers (Z): {[z for z, _ in layers]}\n\n")
            
            f.write("G21\n")
            f.write("G90\n")
            
            # Initial move to safe Z with feedrate
            f.write(f"G0 Z{SAFE_Z} F{RAPID_FEEDRATE:.0f}\n")
            
            if allow_comments:
                f.write("\n")
            
            segment_counter = 0
            continuous_segments = 0
            
            for layer_idx, (layer_z, segments) in enumerate(layers):
                if allow_comments:
                    f.write(f"; {'='*50}\n")
                    f.write(f"; LAYER {layer_idx + 1}/{len(layers)} - Z = {layer_z}\n")
                    f.write(f"; Segments: {len(segments)}\n")
                    f.write(f"; {'='*50}\n\n")
                
                prev_end_x = None
                prev_end_y = None
                
                for seg_idx, (line_num, original_line, start, end, is_reversed, feedrate) in enumerate(segments):
                    segment_counter += 1
                    
                    if is_reversed:
                        actual_start = end
                        actual_end = start
                        direction_note = " [REVERSED]"
                    else:
                        actual_start = start
                        actual_end = end
                        direction_note = ""
                    
                    start_x = actual_start['X']
                    start_y = actual_start['Y']
                    end_x = actual_end['X']
                    end_y = actual_end['Y']
                    
                    # Check if this segment is consecutive (starts where previous ended)
                    is_consecutive = False
                    if prev_end_x is not None and prev_end_y is not None:
                        distance = math.sqrt((start_x - prev_end_x)**2 + (start_y - prev_end_y)**2)
                        if distance < 0.01:  # Tolerance: 0.01mm
                            is_consecutive = True
                            continuous_segments += 1
                    
                    if allow_comments:
                        if is_consecutive:
                            f.write(f"; Segment {segment_counter}{direction_note} F={feedrate:.0f} [CONTINUOUS]\n")
                        else:
                            f.write(f"; Segment {segment_counter}{direction_note} F={feedrate:.0f}\n")
                    
                    if is_consecutive:
                        # Continuous cut - no need to lift Z or move XY
                        f.write(f"G1 X{end_x:.4f} Y{end_y:.4f} F{feedrate:.0f}\n")
                    else:
                        # Non-consecutive - need to lift, move, and lower
                        # Rapid move to segment start (XY at safe Z)
                        f.write(f"G0 X{start_x:.4f} Y{start_y:.4f} F{RAPID_FEEDRATE:.0f}\n")
                        
                        # Lower to work Z with plunge feedrate
                        f.write(f"G1 Z{layer_z:.4f} F{PLUNGE_FEEDRATE:.0f}\n")
                        
                        # Cut movement with segment feedrate
                        f.write(f"G1 X{end_x:.4f} Y{end_y:.4f} F{feedrate:.0f}\n")
                    
                    # Update previous end position
                    prev_end_x = end_x
                    prev_end_y = end_y
                    
                    # Lift to safe Z only if next segment is not consecutive or this is the last segment
                    is_last_in_layer = (seg_idx == len(segments) - 1)
                    next_is_consecutive = False
                    
                    if not is_last_in_layer:
                        next_seg = segments[seg_idx + 1]
                        next_is_reversed = next_seg[4]
                        next_start = next_seg[3] if next_is_reversed else next_seg[2]
                        next_start_x = next_start['X']
                        next_start_y = next_start['Y']
                        distance_to_next = math.sqrt((next_start_x - end_x)**2 + (next_start_y - end_y)**2)
                        if distance_to_next < 0.01:
                            next_is_consecutive = True
                    
                    if is_last_in_layer or not next_is_consecutive:
                        # Lift to safe Z
                        f.write(f"G0 Z{SAFE_Z} F{RAPID_FEEDRATE:.0f}\n")
                    
                    if allow_comments:
                        f.write("\n")
            
            if allow_comments:
                f.write("; End program\n")
            f.write("M30\n")
        
        print(f"✓ Written {segment_counter} segments to {output_filename}")
        if continuous_segments > 0:
            print(f"  ({continuous_segments} continuous segments detected, Z moves optimized)")
        if not allow_comments:
            print("  (comments disabled for compact output)")

# Main
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python gcode_optimizer.py <input_file> [options]")
        print("\nExamples:")
        print("  python gcode_optimizer.py input.gcode")
        print("  python gcode_optimizer.py input.gcode --layer -1.0")
        print("  python gcode_optimizer.py input.gcode --allow-reverse --allow-comments")
        print("  python gcode_optimizer.py input.gcode --force")
        print("\nOptions:")
        print("  --layer <Z>:           Optimize only single layer at Z (default: all layers)")
        print("  --allow-reverse (-r):  Allow segment reversal for better optimization")
        print("  --allow-comments (-c): Include comments in output G-code")
        print("  --force (-f):          Overwrite output file without asking")
        print("  --verbose (-v):        Detailed debug output")
        print("\nConfiguration (edit in script):")
        print(f"  SAFE_Z = {SAFE_Z}")
        print(f"  RAPID_FEEDRATE = {RAPID_FEEDRATE}")
        print(f"  PLUNGE_FEEDRATE = {PLUNGE_FEEDRATE}")
        print(f"  DEFAULT_FEEDRATE = {DEFAULT_FEEDRATE}")
        sys.exit(1)
    
    input_file = sys.argv[1]
    verbose = '--verbose' in sys.argv or '-v' in sys.argv
    allow_reverse = '--allow-reverse' in sys.argv or '-r' in sys.argv
    allow_comments = '--allow-comments' in sys.argv or '-c' in sys.argv
    force_overwrite = '--force' in sys.argv or '-f' in sys.argv
    
    # Check for --layer option
    single_layer = False
    target_z = None
    if '--layer' in sys.argv:
        single_layer = True
        layer_idx = sys.argv.index('--layer')
        if layer_idx + 1 < len(sys.argv):
            try:
                target_z = float(sys.argv[layer_idx + 1])
            except ValueError:
                print("❌ Error: --layer requires a numeric Z value")
                sys.exit(1)
        else:
            print("❌ Error: --layer requires a Z value")
            sys.exit(1)
    
    file_name, file_ext = os.path.splitext(input_file)
    output_file = f"{file_name}-optimized{file_ext}"
    
    try:
        optimizer = GCodeOptimizer(input_file, verbose=verbose)
        # all_layers is True by default, False only if --layer is specified
        optimizer.optimize(target_z, output_file, allow_reverse, not single_layer, allow_comments, force_overwrite)
    except FileNotFoundError:
        print(f"❌ Error: File '{input_file}' not found!")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
