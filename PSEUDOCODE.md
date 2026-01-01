# Pseudocode (WaveForms / AD3 Memristor Training)
// ===== Device Control (Check hardware availability) =====
if Wavegen is not initialized:
    throw error "Please open Wavegen 1"
if Scope is not initialized:
    throw error "Please open Scope 1"
if StaticIO is not initialized:
    throw error "Please open Static I/O and press RUN"

// ===== Parameter Initialization =====
RS = 10000 Ω             // series resistor (10 kΩ)
VREAD_A = +0.06 V        // read voltage for A-side memristors (~ +60 mV pulse)
VREAD_B = -0.06 V        // read voltage for B-side memristors (~ -60 mV pulse)
PW = 1e-3 sec            // half-period for pulses (1 ms)
N_PULSE = 30             // number of training pulses per memristor
VF_SET_A = -0.7 V        // SET pulse amplitude for A-side memristors (~ -0.7 V)
VB_SET_B = +0.3 V        // SET pulse amplitude for B-side memristors (~ +0.3 V)

// Fine-tuning (micro-adjustment) pulse parameters
VFP = -0.1 V   // small negative micro RESET pulse
VGP = +0.1 V   // small positive micro SET pulse
ADJ_STEP = 5   // base number of micro pulses for adjustment
MAX_TRIES = 4  // max correction attempts per pattern

// Stability buffer for polarity changes
VBUFFER = 0.2 V    // buffer pulse amplitude (~ 0.2 V)
N_BUFFER = 3       // number of buffer pulses on polarity change
lastPulseSign = None  // polarity of last training pulse sequence (+1, -1, or 0)

// Helper function to get sign of a value (+1, -1, 0)
function sign(x):
    if x > 0: return 1
    if x < 0: return -1
    return 0

// ===== Memristor DIO Assignments =====
ALL_MEMS = [1, 2, 5, 9, 12, 15]        // DIO labels for 6 memristors
shuffle(ALL_MEMS)                     // randomize order
A_MEMS_ALL = first 3 of ALL_MEMS      // assign 3 random memristors to A-group
B_MEMS_ALL = last 3 of ALL_MEMS       // assign remaining 3 to B-group
shuffle(A_MEMS_ALL)
shuffle(B_MEMS_ALL)
mem_FA = A_MEMS_ALL[0], mem_FB = A_MEMS_ALL[1], mem_FC = A_MEMS_ALL[2]
mem_RB = B_MEMS_ALL[0], mem_RC = B_MEMS_ALL[1], mem_RD = B_MEMS_ALL[2]
MEMS = [mem_FA, mem_FB, mem_FC, mem_RB, mem_RC, mem_RD]
PHASE_OF_MEM = { mem_FA:"FA", mem_FB:"FB", mem_FC:"FC", mem_RB:"RB", mem_RC:"RC", mem_RD:"RD" }

// Initialize measurement counters
for each m in MEMS:
    COUNT[m] = 0

// ===== StaticIO (Digital I/O) control helpers =====
function initPins(labelList):
    # Set each specified DIO pin as output (push-pull) and drive LOW (0)
    for each lbl in labelList:
        configure DIO(lbl) as output (Push-Pull)
        set DIO(lbl) = 0    // all pins off initially
    apply StaticIO configuration

function allOff(labelList):
    # Turn all listed DIO pins LOW (off)
    for each lbl in labelList:
        set DIO(lbl) = 0
    apply StaticIO configuration

function setExclusive(lbl, allList):
    # Turn off all pins, then enable only the specified pin (Break-Before-Make)
    if lbl not in allList:
        print "Warning: Invalid DIO access", lbl
        return
    allOff(allList)
    wait T_BBM                // brief delay (~10 ms) before making new connection
    set DIO(lbl) = 1          // set target pin HIGH (on)
    apply StaticIO configuration
    wait T_EN_SETTLE          // wait for line to settle (~2 ms)

// ===== Wavegen/Scope control helpers =====
function w1_to_dc0():
    # Set Wavegen Channel1 to 0 V DC and stop output
    set Wavegen.Channel1 mode = DC, Offset = 0.0 V
    stop Wavegen output

function w1_square_v(vwrite, pw, n):
    # Generate n pulses of a square wave on Wavegen Channel1
    amplitude = |vwrite| / 2
    offset = vwrite / 2
    freq = 1 / (2 * pw)                    // one pulse period = 2*pw
    configure Wavegen.Channel1 as Square (freq, amplitude, offset)
    run Wavegen output
    wait (n * 2 * pw + small_margin)      // wait for n pulses to complete
    stop Wavegen output

function scopePrep():
    # Prepare oscilloscope for measurement (DC coupling, no trigger yet)
    set Scope.Channel1 coupling = DC, offset = 0
    set Scope.Channel2 coupling = DC, offset = 0
    set Scope.Trigger source = None
    set Scope.Time.Base = 5e-3 sec/div    // time base ~5 ms/div
    enable Scope.Channel2, set Range = 2.0 V (for both channels)

function scopeSingle():
    set Scope to single-trigger mode (arm for one acquisition)

function scopeWait():
    wait for Scope acquisition to complete

function measAvg(channel):
    # Measure average voltage on a Scope channel
    v = Scope.Channel<channel>.measure("mean")
    if v is invalid:
        v = Scope.Channel<channel>.measure("Average")
    return v or 0.0

// ===== Baseline Calibration (0 V reference for each memristor) =====
BASE_V1 = {}, BASE_V2 = {}   // baseline voltage dictionaries for channels
function calibrateBaselineAll():
    print "== BASELINE CALIBRATION =="
    w1_to_dc0()   // ensure 0 V output
    for each mem in MEMS:
        setExclusive(mem, MEMS)         // connect only this memristor
        Wavegen.run (0 V DC output)     // apply 0 V
        wait T_DC_SETTLE               // wait ~1 s for DC to stabilize
        scopePrep()
        scopeSingle()
        scopeWait()                    // capture baseline reading
        stop Wavegen
        V1 = measAvg(1) or 0.0         // average Channel1 voltage
        V2 = measAvg(2) or 0.0         // average Channel2 voltage
        BASE_V1[mem] = V1
        BASE_V2[mem] = V2
        print "MEM", mem, "(", PHASE_OF_MEM[mem], ") baseline: V1_base =", V1, ", V2_base =", V2
        allOff(MEMS)
        w1_to_dc0()
        wait T_RELAX                   // short relax (~10 ms) after each measurement

// ===== Memristor Read (non-destructive measurement) =====
function readMem(mem, phaseLabel, cycleIdx):
    # Apply a short pulse to measure a memristor's conductance
    setExclusive(mem, MEMS)
    if mem is in A_MEMS_ALL:           // A-side memristor
        offsetVal = VREAD_A            // small positive read pulse
        triggerChannel = 1
        triggerSlope = Rising edge
    else:                              // B-side memristor
        offsetVal = VREAD_B            // small negative read pulse
        triggerChannel = 2
        triggerSlope = Falling edge
    scopePrep()
    Scope.Trigger.Source = "Channel" + triggerChannel
    Scope.Trigger.Type = Edge, Scope.Trigger.Level = offsetVal / 2
    Scope.Trigger.Edge.Slope = triggerSlope
    Scope.Time.Base = 2e-3 sec         // narrow time base (~2 ms window)
    # Generate a single read pulse on Wavegen Channel1
    configure Wavegen.Channel1 (Square wave, freq = 1/(2*PW), amp = |offsetVal|/2, offset = offsetVal/2)
    scopeSingle()
    Wavegen.run()                      // output one read pulse
    wait (2 * PW + small_margin)       // wait one pulse period (~2 ms)
    scopeWait()
    stop Wavegen
    # Measure peak voltages on both scope channels during the pulse
    V1_raw = Scope.Channel1.measure("High") or Scope.Channel1.measure("Maximum") or 0.0
    V2_raw = Scope.Channel2.measure("High") or Scope.Channel2.measure("Maximum") or 0.0
    # Subtract baseline to get actual node voltages
    V1c = V1_raw - BASE_V1[mem]
    V2c = V2_raw - BASE_V2[mem]
    if triggerChannel == 1:
        Vnode = V1c    // A-side node voltage
    else:
        Vnode = V2c    // B-side node voltage
    # Calculate current through memristor: I = (V2c - V1c) / RS
    I = (V2c - V1c) / RS
    if |I| < 1e-15:
        I = 1e-15      // avoid division by zero (tiny current instead)
    Rm = |Vnode / I|                 // memristor resistance magnitude
    Gm = (Rm is finite) ? (1 / Rm) : 0  // memristor conductance
    # Log the measurement (phaseLabel, cycleIdx, mem, Vnode, I, Gm)
    print "LOG", phaseLabel, cycleIdx, "MEM" + mem, "Vnode =", Vnode, "I =", I, "G =", Gm
    allOff(MEMS)
    w1_to_dc0()
    wait T_RELAX
    return Gm

// ===== Training (write) and adjustment helpers =====
function measureAndPlot(mem, tag):
    # Measure memristor conductance and increment its count
    G = readMem(mem, tag + "_" + PHASE_OF_MEM[mem], COUNT[mem])
    COUNT[mem] += 1
    return G

function trainMem(mem, vwrite, pulseCount):
    # Apply SET pulses to a memristor and verify conductance increase
    setExclusive(mem, MEMS)
    currentSign = sign(vwrite)
    if lastPulseSign is not None and currentSign != lastPulseSign:
        # If switching pulse polarity, apply buffer pulses for stability
        bufferV = (currentSign > 0) ? +VBUFFER : -VBUFFER
        print "[Stability] Polarity changed, apply", N_BUFFER, "buffer pulses at", bufferV, "V"
        w1_square_v(bufferV, PW, N_BUFFER)
        wait T_AFTER_PULSE
        w1_to_dc0()
        wait T_EN_SETTLE
    # Apply main training pulses
    w1_square_v(vwrite, PW, pulseCount)
    wait T_AFTER_PULSE
    G_after = measureAndPlot(mem, "TRAIN")    // conductance after training
    print "G after training =", G_after
    lastPulseSign = currentSign

function patternSum(pattern, tag):
    # Measure and return sum of conductances for the two memristors corresponding to the pattern
    if pattern == "11":
        return measureAndPlot(mem_FA, tag) + measureAndPlot(mem_FB, tag)
    if pattern == "10":
        return measureAndPlot(mem_FA, tag) + measureAndPlot(mem_RC, tag)
    if pattern == "01":
        return measureAndPlot(mem_RB, tag) + measureAndPlot(mem_FC, tag)
    if pattern == "00":
        return measureAndPlot(mem_RB, tag) + measureAndPlot(mem_RD, tag)

function microPulse(mem, volts, count, label=""):
    # Apply small adjustment pulses to a memristor
    print "->", count, "x", volts, "V micro pulses on", PHASE_OF_MEM[mem], label
    setExclusive(mem, MEMS)
    w1_square_v(volts, PW, count)
    allOff(MEMS)
    w1_to_dc0()
    wait T_AFTER_PULSE

function applyAdjustment(target, detected, count):
    # Apply microPulse corrections based on target vs detected pattern
    if target == "10":
        if detected == "01":
            microPulse(mem_FA, VFP, count, "(FA -0.1V)")
            microPulse(mem_RB, VFP, count, "(RB -0.1V)")
            microPulse(mem_RC, VGP, count, "(RC +0.1V)")
            microPulse(mem_FC, VGP, count, "(FC +0.1V)")
        else if detected == "00":
            microPulse(mem_FA, VFP, count, "(FA -0.1V)")
            microPulse(mem_RB, VFP, count, "(RB -0.1V)")
        else if detected == "11":
            microPulse(mem_FB, VGP, count, "(FB +0.1V)")
            microPulse(mem_RC, VGP, count, "(RC +0.1V)")
    else if target == "11":
        if detected == "00":
            microPulse(mem_FA, VFP, count, "(FA -0.1V)")
            microPulse(mem_FB, VFP, count, "(FB -0.1V)")
            microPulse(mem_RB, VFP, count, "(RB -0.1V)")
            microPulse(mem_RD, VFP, count, "(RD -0.1V)")
        else if detected == "01":
            microPulse(mem_FA, VFP, count, "(FA -0.1V)")
            microPulse(mem_RB, VFP, count, "(RB -0.1V)")
        else if detected == "10":
            microPulse(mem_FB, VFP, count, "(FB -0.1V)")
            microPulse(mem_RC, VFP, count, "(RC -0.1V)")
    else if target == "00":
        if detected == "11":
            microPulse(mem_RB, VGP, count, "(RB +0.1V)")
            microPulse(mem_RD, VGP, count, "(RD +0.1V)")
            microPulse(mem_FA, VGP, count, "(FA +0.1V)")
            microPulse(mem_FB, VGP, count, "(FB +0.1V)")
        else if detected == "01":
            microPulse(mem_RD, VGP, count, "(RD +0.1V)")
            microPulse(mem_FC, VFP, count, "(FC -0.1V)")
        else if detected == "10":
            microPulse(mem_RB, VGP, count, "(RB +0.1V)")
            microPulse(mem_FA, VGP, count, "(FA +0.1V)")
    else if target == "01":
        if detected == "10":
            microPulse(mem_RB, VGP, count, "(RB +0.1V)")
            microPulse(mem_FA, VGP, count, "(FA +0.1V)")
            microPulse(mem_FC, VFP, count, "(FC -0.1V)")
            microPulse(mem_RC, VFP, count, "(RC -0.1V)")
        else if detected == "11":
            microPulse(mem_RB, VGP, count, "(RB +0.1V)")
            microPulse(mem_FA, VGP, count, "(FA +0.1V)")
        else if detected == "00":
            microPulse(mem_FC, VFP, count, "(FC -0.1V)")
            microPulse(mem_RD, VFP, count, "(RD -0.1V)")

// ===== Data sets: training, validation, test patterns =====
trainingData    = "101111001001110010011001110010011001001100011011"
validationData  = "1110010010110001"
testData        = "0111100011100100"
function getPairs(bitString):
    pairs = []
    for i from 0 to length(bitString) - 1 step 2:
        pairs.append(bitString[i : i+2])
    return pairs

trainingPatterns   = getPairs(trainingData)
validationPatterns = getPairs(validationData)
testPatterns       = getPairs(testData)
meaning = {
    "11": "Right Eye Blink",
    "10": "Left Eye Blink",
    "00": "No Movement",
    "01": "Tongue Out"
}

// ===== Training Phase =====
print "== TRAINING START =="
for idx, pat in enumerate(trainingPatterns):
    print "-- Training", (idx+1), "/", len(trainingPatterns), "Pattern", pat, "(", meaning[pat], ")"
    b1 = pat[0], b2 = pat[1]
    if b1 == '1':
        print "Bit1=1 -> Train MEM", mem_FA, "(FA) with", N_PULSE, "pulses of VF_SET_A"
        trainMem(mem_FA, VF_SET_A, N_PULSE)
        if b2 == '1':
            print "Bit2=1 -> Train MEM", mem_FB, "(FB) with", N_PULSE, "pulses of VF_SET_A"
            trainMem(mem_FB, VF_SET_A, N_PULSE)
        else:
            print "Bit2=0 -> Train MEM", mem_RC, "(RC) with", N_PULSE, "pulses of VB_SET_B"
            trainMem(mem_RC, VB_SET_B, N_PULSE)
    else:
        print "Bit1=0 -> Train MEM", mem_RB, "(RB) with", N_PULSE, "pulses of VB_SET_B"
        trainMem(mem_RB, VB_SET_B, N_PULSE)
        if b2 == '1':
            print "Bit2=1 -> Train MEM", mem_FC, "(FC) with", N_PULSE, "pulses of VF_SET_A"
            trainMem(mem_FC, VF_SET_A, N_PULSE)
        else:
            print "Bit2=0 -> Train MEM", mem_RD, "(RD) with", N_PULSE, "pulses of VB_SET_B"
            trainMem(mem_RD, VB_SET_B, N_PULSE)
    # After training each pattern, turn off all pins and reset output
    allOff(MEMS)
    w1_to_dc0()
    wait T_AFTER_PULSE   // wait ~100 ms for memristors to settle
print "== TRAINING COMPLETE =="

// Post-training baseline and conductance measurements
calibrateBaselineAll()
print "== POST-TRAINING CONDUCTANCE =="
initialVals = {}
currentG    = {}
for each mem in MEMS:
    nm = PHASE_OF_MEM[mem]
    G0 = readMem(mem, nm, 0)
    initialVals[mem] = G0
    COUNT[mem] += 1
    print nm, "[0] =", G0
for each mem in MEMS:
    currentG[mem] = measureAndPlot(mem, "POST")
    print PHASE_OF_MEM[mem], "(final) =", currentG[mem]
targetSum = {}
targetSum["11"] = currentG[mem_FA] + currentG[mem_FB]   // FA + FB
targetSum["10"] = currentG[mem_FA] + currentG[mem_RC]   // FA + RC
targetSum["00"] = currentG[mem_RB] + currentG[mem_RD]   // RB + RD
targetSum["01"] = currentG[mem_RB] + currentG[mem_FC]   // RB + FC
print "Target sum 11 =", targetSum["11"]
print "Target sum 10 =", targetSum["10"]
print "Target sum 00 =", targetSum["00"]
print "Target sum 01 =", targetSum["01"]

// ===== Validation Phase =====
print "== VALIDATION PHASE =="
for idx, pat in enumerate(validationPatterns):
    print "-- Validation", (idx+1), "/", len(validationPatterns), "Pattern", pat, "(", meaning[pat], ")"
    b1 = pat[0], b2 = pat[1]
    # Measure conductance sum for this pattern
    valSum = 0
    if b1 == '1':
        valSum += measureAndPlot(mem_FA, "VAL")
        if b2 == '1':
            valSum += measureAndPlot(mem_FB, "VAL")
        else:
            valSum += measureAndPlot(mem_RC, "VAL")
    else:
        valSum += measureAndPlot(mem_RB, "VAL")
        if b2 == '1':
            valSum += measureAndPlot(mem_FC, "VAL")
        else:
            valSum += measureAndPlot(mem_RD, "VAL")
    # Determine closest pattern by comparing to targetSum
    detected = None
    minDiff = ∞
    for each p in ["11","10","01","00"]:
        diff = |valSum - targetSum[p]|
        if diff < minDiff:
            minDiff = diff
            detected = p
    print "Detected pattern:", detected, "(Δ=", minDiff, ")"
    if detected == pat:
        print "Correct: pattern", pat, "recognized."
    else:
        print "Incorrect (expected", pat, ", got", detected, "). Applying correction..."
        corrected = false
        currentDetected = detected
        for attempt from 1 to MAX_TRIES:
            pulseCount = ADJ_STEP * attempt
            print "Adjustment attempt", attempt, "-", pulseCount, "micro pulses"
            applyAdjustment(pat, currentDetected, pulseCount)
            # Measure again after adjustment
            newSum = 0
            if b1 == '1':
                newSum += measureAndPlot(mem_FA, "VAL_TRY" + attempt)
                if b2 == '1':
                    newSum += measureAndPlot(mem_FB, "VAL_TRY" + attempt)
                else:
                    newSum += measureAndPlot(mem_RC, "VAL_TRY" + attempt)
            else:
                newSum += measureAndPlot(mem_RB, "VAL_TRY" + attempt)
                if b2 == '1':
                    newSum += measureAndPlot(mem_FC, "VAL_TRY" + attempt)
                else:
                    newSum += measureAndPlot(mem_RD, "VAL_TRY" + attempt)
            # Determine new detected pattern
            currentDetected = None
            newMinDiff = ∞
            for each p in ["11","10","01","00"]:
                dN = |newSum - targetSum[p]|
                if dN < newMinDiff:
                    newMinDiff = dN
                    currentDetected = p
            print "New detected:", currentDetected, "(Δ=", newMinDiff, ")"
            if currentDetected == pat:
                print "Corrected: pattern", pat, "now recognized."
                corrected = true
                break    // exit adjustment loop
            else:
                if attempt < MAX_TRIES:
                    print "Still incorrect, trying again..."
                else:
                    print "Max attempts reached, could not correct."
        if not corrected:
            print "Still wrong (expected", pat, ", got", currentDetected, "). Continuing to next."
print "== VALIDATION COMPLETE =="

// ===== Test Phase (final test) =====
print "== TEST PHASE =="
for idx, pat in enumerate(testPatterns):
    print "-- Test", (idx+1), "/", len(testPatterns), "Pattern", pat, "(", meaning[pat], ")"
    b1 = pat[0], b2 = pat[1]
    testSum = 0
    if b1 == '1':
        testSum += measureAndPlot(mem_FA, "TEST")
        if b2 == '1':
            testSum += measureAndPlot(mem_FB, "TEST")
        else:
            testSum += measureAndPlot(mem_RC, "TEST")
    else:
        testSum += measureAndPlot(mem_RB, "TEST")
        if b2 == '1':
            testSum += measureAndPlot(mem_FC, "TEST")
        else:
            testSum += measureAndPlot(mem_RD, "TEST")
    detected = None
    minDiff = ∞
    for each p in ["11","10","01","00"]:
        diff = |testSum - targetSum[p]|
        if diff < minDiff:
            minDiff = diff
            detected = p
    print "Detected pattern:", detected, "(Δ=", minDiff, ")"
    if detected == pat:
        print "Correct:", pat, "recognized."
    else:
        print "Incorrect (expected", pat, ", got", detected, ")."
print "== TEST COMPLETE =="
print "Finished. All steps completed."

