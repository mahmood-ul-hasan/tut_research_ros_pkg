## Pose Graph Optimization using g2o Parameters

- **chi2 (Chi-Squared Error):** A lower chi-squared value indicates better alignment between estimated poses and observed measurements. It represents the difference between predicted measurements (based on estimated poses) and actual observations. Smaller values indicate reduced error.

- **lambda (Levenberg-Marquardt Parameter):** Lambda affects convergence speed and stability. High values make optimization cautious, leading to slow convergence. Low values might cause divergence. The optimal value depends on the problem's nature.

- **levenbergIter (Levenberg-Marquardt Iterations):** The number of iterations the algorithm adjusts estimated poses to minimize chi-squared error. More iterations are needed for complex or poorly-initialized problems. Few iterations may indicate quick convergence, while more might indicate challenging convergence.

- **schur (Schur Complement Step):** The presence of a Schur step indicates using a technique to improve optimization. Schur complement solves a smaller subproblem efficiently instead of the entire problem. Beneficial for optimizing large graphs.

In summary, a successful pose graph optimization yields low chi-squared error, aligning estimated poses with observations. Lambda and iterations balance convergence speed and stability, and the Schur step enhances efficiency. Interpretation also relies on domain knowledge.



# A lower covariance matrix, typically indicates lower uncertainty 
# Information Matrix = 1 / Covariance Matrix
# A higher information matrix indicates lower uncertainty

