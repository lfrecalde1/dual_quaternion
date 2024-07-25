/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_quadrotor.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#define NX     QUADROTOR_NX
#define NZ     QUADROTOR_NZ
#define NU     QUADROTOR_NU
#define NP     QUADROTOR_NP
#define NBX    QUADROTOR_NBX
#define NBX0   QUADROTOR_NBX0
#define NBU    QUADROTOR_NBU
#define NSBX   QUADROTOR_NSBX
#define NSBU   QUADROTOR_NSBU
#define NSH    QUADROTOR_NSH
#define NSG    QUADROTOR_NSG
#define NSPHI  QUADROTOR_NSPHI
#define NSHN   QUADROTOR_NSHN
#define NSGN   QUADROTOR_NSGN
#define NSPHIN QUADROTOR_NSPHIN
#define NSBXN  QUADROTOR_NSBXN
#define NS     QUADROTOR_NS
#define NSN    QUADROTOR_NSN
#define NG     QUADROTOR_NG
#define NBXN   QUADROTOR_NBXN
#define NGN    QUADROTOR_NGN
#define NY0    QUADROTOR_NY0
#define NY     QUADROTOR_NY
#define NYN    QUADROTOR_NYN
#define NH     QUADROTOR_NH
#define NPHI   QUADROTOR_NPHI
#define NHN    QUADROTOR_NHN
#define NPHIN  QUADROTOR_NPHIN
#define NR     QUADROTOR_NR


int main()
{

    quadrotor_solver_capsule *acados_ocp_capsule = quadrotor_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = QUADROTOR_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = quadrotor_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("quadrotor_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = quadrotor_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = quadrotor_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = quadrotor_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = quadrotor_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = quadrotor_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = quadrotor_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    int idxbx0[NBX0];
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;
    idxbx0[12] = 12;
    idxbx0[13] = 13;

    double lbx0[NBX0];
    double ubx0[NBX0];
    lbx0[0] = 0.01747472219715289;
    ubx0[0] = 0.01747472219715289;
    lbx0[1] = -0.4392853214756301;
    ubx0[1] = -0.4392853214756301;
    lbx0[2] = -0.06036243373217249;
    ubx0[2] = -0.06036243373217249;
    lbx0[3] = 0.8961469840456496;
    ubx0[3] = 0.8961469840456496;
    lbx0[4] = -1.6016350254555862;
    ubx0[4] = -1.6016350254555862;
    lbx0[5] = 1.0509275975725676;
    ubx0[5] = 1.0509275975725676;
    lbx0[6] = -0.9843360861475858;
    ubx0[6] = -0.9843360861475858;
    lbx0[7] = 0.4800867274761232;
    ubx0[7] = 0.4800867274761232;
    lbx0[8] = 0;
    ubx0[8] = 0;
    lbx0[9] = 0;
    ubx0[9] = 0;
    lbx0[10] = 0;
    ubx0[10] = 0;
    lbx0[11] = 0;
    ubx0[11] = 0;
    lbx0[12] = 0;
    ubx0[12] = 0;
    lbx0[13] = 0;
    ubx0[13] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // initialization for state values
    double x_init[NX];
    x_init[0] = 0.0;
    x_init[1] = 0.0;
    x_init[2] = 0.0;
    x_init[3] = 0.0;
    x_init[4] = 0.0;
    x_init[5] = 0.0;
    x_init[6] = 0.0;
    x_init[7] = 0.0;
    x_init[8] = 0.0;
    x_init[9] = 0.0;
    x_init[10] = 0.0;
    x_init[11] = 0.0;
    x_init[12] = 0.0;
    x_init[13] = 0.0;

    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;
    // set parameters
    double p[NP];
    p[0] = 0;
    p[1] = 0;
    p[2] = 0;
    p[3] = 0;
    p[4] = 0;
    p[5] = 0;
    p[6] = 0;
    p[7] = 0;
    p[8] = 0;
    p[9] = 0;
    p[10] = 0;
    p[11] = 0;
    p[12] = 0;
    p[13] = 0;
    p[14] = 0;
    p[15] = 0;
    p[16] = 0;
    p[17] = 0;

    for (int ii = 0; ii <= N; ii++)
    {
        quadrotor_acados_update_params(acados_ocp_capsule, ii, p, NP);
    }
  

    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)];
    double utraj[NU * N];


    // solve ocp in loop
    int rti_phase = 0;

    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = quadrotor_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( NX, N+1, xtraj, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( NU, N, utraj, NU );
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("quadrotor_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("quadrotor_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    quadrotor_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

    // free solver
    status = quadrotor_acados_free(acados_ocp_capsule);
    if (status) {
        printf("quadrotor_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = quadrotor_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("quadrotor_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
}
