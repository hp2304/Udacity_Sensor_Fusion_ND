# SFND Radar Target Detection

## CFAR Calculation

```matlab
nb_rows = size(RDM, 1);
nb_cols = size(RDM, 2);
CFAR = zeros(nb_rows, nb_cols);

% number of training cells in the window
nb_window_training_cells = (2*Tr+2*Gr+1)*(2*Td+2*Gd+1)-(2*Gr+1)*(2*Gd+1);

% save RDM linear (power) for easy lookup
RDM_linear = db2pow(RDM);

% loop through non edge locations
for i=(Tr + Gr + 1):(nb_rows - (Tr + Gr))
    for j=(Td + Gd + 1):(nb_cols - (Td + Gd))
        
        % window bounds calculation
        row_start = i - (Tr + Gr);
        row_end = i + (Tr + Gr);
        col_start = j - (Td + Gd);
        col_end = j + (Td + Gd);
        window = RDM_linear(row_start:row_end, col_start:col_end);

        guard_window = RDM_linear(i - Gr: i + Gr, j - Gd: j + Gd);
        
        % from the sum all elements of whole window subtract
        % the sum all elements of guard window,
        % it gives the sum of all training cells
        % take average by dividing it by the qty of the same
        th = (sum(window, 'all') - sum(guard_window, 'all')) / nb_window_training_cells;
        
        % convert to dB and add offset_dB
        th_dB = pow2db(th);
        th_dB = th_dB + offset_dB;

        % if RDM is higher than threshold then assign 1
        if RDM(i ,j) > th_dB
            CFAR(i, j) = 1;
        end
    end
end
```

- *CFAR* variable is initialized as matrix of zeros, same size as RDM.
- Number of training cells in the window is fixed, hence it's calculated and stored for later use.

- RDM's dB calculation is converted to linear power unit to avoid extraneous ```db2pow``` calls.

- A nested loop structure,

```matlab
for i=(Tr + Gr + 1):(nb_rows - (Tr + Gr))
    for j=(Td + Gd + 1):(nb_cols - (Td + Gd))
```

start and end for both row and column are defined as above because CFAR calculation is only valid within these bounds. Any location outside these limits would be invalid since imposing the window on CUT would fall outside the RDM matrix.

- A 2D window centered at CUT is selected using indexing on RDM (*linear*) matrix. Also, a 2D guard window is selected similarly.

```
row_start = i - (Tr + Gr);
row_end = i + (Tr + Gr);
col_start = j - (Td + Gd);
col_end = j + (Td + Gd);
window = RDM_linear(row_start:row_end, col_start:col_end);

guard_window = RDM_linear(i - Gr: i + Gr, j - Gd: j + Gd);
```

- Sum of all elements of guard window is subtracted from the sum of all elements of whole window. Such gives the sum of training cell entries. Then, it's divided by total number training cells in the window (stored previously) to get average. Calculated value is the dynamic threshold.

```
th = (sum(window, 'all') - sum(guard_window, 'all')) / nb_window_training_cells;
```

- Aforementioned is then converted to dB and an offset value is added to it.

```
th_dB = pow2db(th);
th_dB = th_dB + offset_dB;
```

- CFAR value is set to 1 if RDM value (in dB) is above the previously calculated threshold.

```
if RDM(i ,j) > th_dB
    CFAR(i, j) = 1;
end
```