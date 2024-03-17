import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches
plt.rc('text', usetex = True)
def fancy_plots_2():
    # Define parameters fancy plot
    pts_per_inch = 72.27
    # write "\the\textwidth" (or "\showthe\columnwidth" for a 2 collumn text)
    text_width_in_pts = 300.0
    # inside a figure environment in latex, the result will be on the
    # dvi/pdf next to the figure. See url above.
    text_width_in_inches = text_width_in_pts / pts_per_inch
    # make rectangles with a nice proportion
    golden_ratio = 0.618
    # figure.png or figure.eps will be intentionally larger, because it is prettier
    inverse_latex_scale = 2
    # when compiling latex code, use
    # \includegraphics[scale=(1/inverse_latex_scale)]{figure}
    # we want the figure to occupy 2/3 (for example) of the text width
    fig_proportion = (3.0 / 3.0)
    csize = inverse_latex_scale * fig_proportion * text_width_in_inches
    # always 1.0 on the first argument
    fig_size = (1.0 * csize, 0.7 * csize)
    # find out the fontsize of your latex text, and put it here
    text_size = inverse_latex_scale * 10
    label_size = inverse_latex_scale * 10
    tick_size = inverse_latex_scale * 8

    params = {'backend': 'ps',
            'axes.labelsize': text_size,
            'legend.fontsize': tick_size,
            'legend.handlelength': 2.5,
            'legend.borderaxespad': 0,
            'xtick.labelsize': tick_size,
            'ytick.labelsize': tick_size,
            'font.family': 'serif',
            'font.size': text_size,
            # Times, Palatino, New Century Schoolbook,
            # Bookman, Computer Modern Roman
            # 'font.serif': ['Times'],
            'ps.usedistiller': 'xpdf',
            'text.usetex': True,
            'figure.figsize': fig_size,
            # include here any neede package for latex
            'text.latex.preamble': [r'\usepackage{amsmath}',
                ],
                }
    plt.rc(params)
    plt.clf()
    # figsize accepts only inches.
    fig = plt.figure(1, figsize=fig_size)
    fig.subplots_adjust(left=0.13, right=0.98, top=0.97, bottom=0.13,
                        hspace=0.05, wspace=0.02)
    plt.ioff()
    ax1 = fig.add_subplot(211)
    ax2 = fig.add_subplot(212)

    return fig, ax1, ax2


def fancy_plots_4():
    # Define parameters fancy plot
    pts_per_inch = 72.27
    # write "\the\textwidth" (or "\showthe\columnwidth" for a 2 collumn text)
    text_width_in_pts = 300.0
    # inside a figure environment in latex, the result will be on the
    # dvi/pdf next to the figure. See url above.
    text_width_in_inches = text_width_in_pts / pts_per_inch
    # make rectangles with a nice proportion
    golden_ratio = 0.618
    # figure.png or figure.eps will be intentionally larger, because it is prettier
    inverse_latex_scale = 2
    # when compiling latex code, use
    # \includegraphics[scale=(1/inverse_latex_scale)]{figure}
    # we want the figure to occupy 2/3 (for example) of the text width
    fig_proportion = (3.0 / 3.0)
    csize = inverse_latex_scale * fig_proportion * text_width_in_inches
    # always 1.0 on the first argument
    fig_size = (1.0 * csize, 0.7 * csize)
    # find out the fontsize of your latex text, and put it here
    text_size = inverse_latex_scale * 10
    label_size = inverse_latex_scale * 10
    tick_size = inverse_latex_scale * 8

    params = {'backend': 'ps',
            'axes.labelsize': text_size,
            'legend.fontsize': tick_size,
            'legend.handlelength': 2.5,
            'legend.borderaxespad': 0,
            'xtick.labelsize': tick_size,
            'ytick.labelsize': tick_size,
            'font.family': 'serif',
            'font.size': text_size,
            # Times, Palatino, New Century Schoolbook,
            # Bookman, Computer Modern Roman
            # 'font.serif': ['Times'],
            'ps.usedistiller': 'xpdf',
            'text.usetex': True,
            'figure.figsize': fig_size,
            # include here any neede package for latex
            'text.latex.preamble': [r'\usepackage{amsmath}',
                ],
                }
    plt.rc(params)
    plt.clf()
    # figsize accepts only inches.
    fig = plt.figure(1, figsize=fig_size)
    fig.subplots_adjust(left=0.13, right=0.98, top=0.97, bottom=0.13,
                        hspace=0.05, wspace=0.02)
    plt.ioff()
    ax1 = fig.add_subplot(411)
    ax2 = fig.add_subplot(412)
    ax3 = fig.add_subplot(413)
    ax4 = fig.add_subplot(414)

    return fig, ax1, ax2, ax3, ax4

def fancy_plots_3():
    # Define parameters fancy plot
    pts_per_inch = 72.27
    # write "\the\textwidth" (or "\showthe\columnwidth" for a 2 collumn text)
    text_width_in_pts = 300.0
    # inside a figure environment in latex, the result will be on the
    # dvi/pdf next to the figure. See url above.
    text_width_in_inches = text_width_in_pts / pts_per_inch
    # make rectangles with a nice proportion
    golden_ratio = 0.618
    # figure.png or figure.eps will be intentionally larger, because it is prettier
    inverse_latex_scale = 2
    # when compiling latex code, use
    # \includegraphics[scale=(1/inverse_latex_scale)]{figure}
    # we want the figure to occupy 2/3 (for example) of the text width
    fig_proportion = (3.0 / 3.0)
    csize = inverse_latex_scale * fig_proportion * text_width_in_inches
    # always 1.0 on the first argument
    fig_size = (1.0 * csize, 0.7 * csize)
    # find out the fontsize of your latex text, and put it here
    text_size = inverse_latex_scale * 10
    label_size = inverse_latex_scale * 10
    tick_size = inverse_latex_scale * 8

    params = {'backend': 'ps',
            'axes.labelsize': text_size,
            'legend.fontsize': tick_size,
            'legend.handlelength': 2.5,
            'legend.borderaxespad': 0,
            'xtick.labelsize': tick_size,
            'ytick.labelsize': tick_size,
            'font.family': 'serif',
            'font.size': text_size,
            # Times, Palatino, New Century Schoolbook,
            # Bookman, Computer Modern Roman
            # 'font.serif': ['Times'],
            'ps.usedistiller': 'xpdf',
            'text.usetex': True,
            'figure.figsize': fig_size,
            # include here any neede package for latex
            'text.latex.preamble': [r'\usepackage{amsmath}',
                ],
                }
    plt.rc(params)
    plt.clf()
    # figsize accepts only inches.
    fig = plt.figure(1, figsize=fig_size)
    fig.subplots_adjust(left=0.13, right=0.98, top=0.97, bottom=0.13,
                        hspace=0.05, wspace=0.02)
    plt.ioff()
    ax1 = fig.add_subplot(311)
    ax2 = fig.add_subplot(312)
    ax3 = fig.add_subplot(313)

    return fig, ax1, ax2, ax3

def fancy_plots_1():
    # Define parameters fancy plot
    pts_per_inch = 72.27
    # write "\the\textwidth" (or "\showthe\columnwidth" for a 2 collumn text)
    text_width_in_pts = 300.0
    # inside a figure environment in latex, the result will be on the
    # dvi/pdf next to the figure. See url above.
    text_width_in_inches = text_width_in_pts / pts_per_inch
    # make rectangles with a nice proportion
    golden_ratio = 0.618
    # figure.png or figure.eps will be intentionally larger, because it is prettier
    inverse_latex_scale = 2
    # when compiling latex code, use
    # \includegraphics[scale=(1/inverse_latex_scale)]{figure}
    # we want the figure to occupy 2/3 (for example) of the text width
    fig_proportion = (3.0 / 3.0)
    csize = inverse_latex_scale * fig_proportion * text_width_in_inches
    # always 1.0 on the first argument
    fig_size = (1.0 * csize, 0.7 * csize)
    # find out the fontsize of your latex text, and put it here
    text_size = inverse_latex_scale * 10
    label_size = inverse_latex_scale * 10
    tick_size = inverse_latex_scale * 8

    params = {'backend': 'ps',
            'axes.labelsize': text_size,
            'legend.fontsize': tick_size,
            'legend.handlelength': 2.5,
            'legend.borderaxespad': 0,
            'xtick.labelsize': tick_size,
            'ytick.labelsize': tick_size,
            'font.family': 'serif',
            'font.size': text_size,
            # Times, Palatino, New Century Schoolbook,
            # Bookman, Computer Modern Roman
            # 'font.serif': ['Times'],
            'ps.usedistiller': 'xpdf',
            'text.usetex': True,
            'figure.figsize': fig_size,
            # include here any neede package for latex
            'text.latex.preamble': [r'\usepackage{amsmath}',
                ],
                }
    plt.rc(params)
    plt.clf()
    # figsize accepts only inches.
    fig = plt.figure(1, figsize=fig_size)
    fig.subplots_adjust(left=0.13, right=0.98, top=0.97, bottom=0.13,
                        hspace=0.05, wspace=0.02)
    plt.ioff()
    ax1 = fig.add_subplot(111)
    return fig, ax1


def plot_states_position(fig11, ax11, ax21, ax31, x, xd, t, name):
        t = t[0:x.shape[1]]
        ax11.set_xlim((t[0], t[-1]))
        ax21.set_xlim((t[0], t[-1]))
        ax31.set_xlim((t[0], t[-1]))

        ax11.set_xticklabels([])
        ax21.set_xticklabels([])
        state_1_e, = ax11.plot(t[0:t.shape[0]], x[0, 0:t.shape[0]],
                    color='#C43C29', lw=1.0, ls="-")

        state_1_e_d, = ax11.plot(t[0:t.shape[0]], xd[0, 0:t.shape[0]],
                    color='#1D2121', lw=1.0, ls="--")

        state_2_e, = ax21.plot(t[0:t.shape[0]], x[1, 0:t.shape[0]],
                        color='#3FB454', lw=1.0, ls="-")

        state_2_e_d, = ax21.plot(t[0:t.shape[0]], xd[1, 0:t.shape[0]],
                    color='#1D2121', lw=1.0, ls="--")

        state_3_e, = ax31.plot(t[0:t.shape[0]], x[2, 0:t.shape[0]],
                        color='#3F8BB4', lw=1.0, ls="-")

        state_3_e_d, = ax31.plot(t[0:t.shape[0]], xd[2, 0:t.shape[0]],
                    color='#1D2121', lw=1.0, ls="--")

        ax11.set_ylabel(r"$[m]$", rotation='vertical')
        ax11.legend([state_1_e, state_1_e_d],
                [ r'$x$', r'$x_d$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax11.grid(color='#949494', linestyle='-.', linewidth=0.5)

        ax21.set_ylabel(r"$[m]$", rotation='vertical')
        ax21.legend([state_2_e, state_2_e_d],
                [r'$y$', r'$y_d$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax21.grid(color='#949494', linestyle='-.', linewidth=0.5)
        ax21.set_xticklabels([])
    
        ax31.set_ylabel(r"$[m]$", rotation='vertical')
        ax31.legend([state_3_e, state_3_e_d],
                [r'$z$', r'$z$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax31.grid(color='#949494', linestyle='-.', linewidth=0.5)
        ax31.set_xlabel(r"$\textrm{Time}[s]$", labelpad=5)

        fig11.savefig(name + ".pdf")
        fig11.savefig(name + ".png")
        return None

def plot_states_quaternion(fig11, ax11, ax21, ax31, ax41, x, xd, t, name):
        t = t[0:x.shape[1]]
        ax11.set_xlim((t[0], t[-1]))
        ax21.set_xlim((t[0], t[-1]))
        ax31.set_xlim((t[0], t[-1]))
        ax41.set_xlim((t[0], t[-1]))

        ax11.set_xticklabels([])
        ax21.set_xticklabels([])
        ax31.set_xticklabels([])

        state_1_e, = ax11.plot(t[0:t.shape[0]], x[0, 0:t.shape[0]],
                    color='#C43C29', lw=1.0, ls="-")

        state_1_e_d, = ax11.plot(t[0:t.shape[0]], xd[0, 0:t.shape[0]],
                    color='#1D2121', lw=1.0, ls="--")

        state_2_e, = ax21.plot(t[0:t.shape[0]], x[1, 0:t.shape[0]],
                        color='#3FB454', lw=1.0, ls="-")

        state_2_e_d, = ax21.plot(t[0:t.shape[0]], xd[1, 0:t.shape[0]],
                    color='#1D2121', lw=1.0, ls="--")

        state_3_e, = ax31.plot(t[0:t.shape[0]], x[2, 0:t.shape[0]],
                        color='#3F8BB4', lw=1.0, ls="-")

        state_3_e_d, = ax31.plot(t[0:t.shape[0]], xd[2, 0:t.shape[0]],
                    color='#1D2121', lw=1.0, ls="--")

        state_4_e, = ax41.plot(t[0:t.shape[0]], x[3, 0:t.shape[0]],
                        color='#36323E', lw=1.0, ls="-")

        state_4_e_d, = ax41.plot(t[0:t.shape[0]], xd[3, 0:t.shape[0]],
                    color='#1D2121', lw=1.0, ls="--")

        ax11.set_ylabel(r"$[]$", rotation='vertical')
        ax11.legend([state_1_e, state_1_e_d],
                [ r'$q_w$', r'$q_{wd}$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax11.grid(color='#949494', linestyle='-.', linewidth=0.5)

        ax21.set_ylabel(r"$[]$", rotation='vertical')
        ax21.legend([state_2_e, state_2_e_d],
                [r'$q_1$', r'$q_{1d}$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax21.grid(color='#949494', linestyle='-.', linewidth=0.5)
        ax21.set_xticklabels([])

        ax31.set_ylabel(r"$[]$", rotation='vertical')
        ax31.legend([state_3_e, state_3_e_d],
                [r'$q_2$', r'$q_{2d}$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax31.grid(color='#949494', linestyle='-.', linewidth=0.5)
        ax31.set_xticklabels([])
    
        ax41.set_ylabel(r"$[]$", rotation='vertical')
        ax41.legend([state_4_e, state_4_e_d],
                [r'$q_3$', r'$q_{3d}$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax41.grid(color='#949494', linestyle='-.', linewidth=0.5)
        ax41.set_xlabel(r"$\textrm{Time}[s]$", labelpad=5)

        fig11.savefig(name + ".pdf")
        fig11.savefig(name + ".png")
        return None
        
def plot_control_actions(fig11, ax11, ax21, ax31, ax41, F, M, t, name):
        t = t[0:M.shape[1]]
        ax11.set_xlim((t[0], t[-1]))
        ax21.set_xlim((t[0], t[-1]))
        ax31.set_xlim((t[0], t[-1]))
        ax41.set_xlim((t[0], t[-1]))

        ax11.set_xticklabels([])
        ax21.set_xticklabels([])
        ax31.set_xticklabels([])

        state_1_e, = ax11.plot(t[0:t.shape[0]], F[0, 0:t.shape[0]],
                    color='#C43C29', lw=1.0, ls="-")

        state_2_e, = ax21.plot(t[0:t.shape[0]], M[0, 0:t.shape[0]],
                        color='#3FB454', lw=1.0, ls="-")

        state_3_e, = ax31.plot(t[0:t.shape[0]], M[1, 0:t.shape[0]],
                        color='#3F8BB4', lw=1.0, ls="-")

        state_4_e, = ax41.plot(t[0:t.shape[0]], M[2, 0:t.shape[0]],
                        color='#36323E', lw=1.0, ls="-")

        ax11.set_ylabel(r"$[N]$", rotation='vertical')
        ax11.legend([state_1_e],
                [ r'$f_z$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax11.grid(color='#949494', linestyle='-.', linewidth=0.5)

        ax21.set_ylabel(r"$[N.m]$", rotation='vertical')
        ax21.legend([state_2_e],
                [r'$\tau_x$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax21.grid(color='#949494', linestyle='-.', linewidth=0.5)
        ax21.set_xticklabels([])

        ax31.set_ylabel(r"$[N.m]$", rotation='vertical')
        ax31.legend([state_3_e],
                [r'$\tau_y$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax31.grid(color='#949494', linestyle='-.', linewidth=0.5)
        ax31.set_xticklabels([])
    
        ax41.set_ylabel(r"$[N.m]$", rotation='vertical')
        ax41.legend([state_4_e],
                [r'$\tau_z$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax41.grid(color='#949494', linestyle='-.', linewidth=0.5)
        ax41.set_xlabel(r"$\textrm{Time}[s]$", labelpad=5)

        fig11.savefig(name + ".pdf")
        fig11.savefig(name + ".png")
        return None

def plot_states_euler(fig11, ax11, ax21, ax31, x, xd, t, name):
        t = t[0:x.shape[1]]
        ax11.set_xlim((t[0], t[-1]))
        ax21.set_xlim((t[0], t[-1]))
        ax31.set_xlim((t[0], t[-1]))

        ax11.set_xticklabels([])
        ax21.set_xticklabels([])
        state_1_e, = ax11.plot(t[0:t.shape[0]], x[2, 0:t.shape[0]],
                    color='#C43C29', lw=1.0, ls="-")

        state_1_e_d, = ax11.plot(t[0:t.shape[0]], xd[0, 0:t.shape[0]],
                    color='#1D2121', lw=1.0, ls="--")

        state_2_e, = ax21.plot(t[0:t.shape[0]], x[1, 0:t.shape[0]],
                        color='#3FB454', lw=1.0, ls="-")

        state_2_e_d, = ax21.plot(t[0:t.shape[0]], xd[1, 0:t.shape[0]],
                    color='#1D2121', lw=1.0, ls="--")

        state_3_e, = ax31.plot(t[0:t.shape[0]], x[0, 0:t.shape[0]],
                        color='#3F8BB4', lw=1.0, ls="-")

        state_3_e_d, = ax31.plot(t[0:t.shape[0]], xd[2, 0:t.shape[0]],
                    color='#1D2121', lw=1.0, ls="--")

        ax11.set_ylabel(r"$[rad]$", rotation='vertical')
        ax11.legend([state_1_e, state_1_e_d],
                [ r'$\phi$', r'$\phi_d$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax11.grid(color='#949494', linestyle='-.', linewidth=0.5)

        ax21.set_ylabel(r"$[rad]$", rotation='vertical')
        ax21.legend([state_2_e, state_2_e_d],
                [r'$\theta$', r'$\theta_d$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax21.grid(color='#949494', linestyle='-.', linewidth=0.5)
        ax21.set_xticklabels([])
    
        ax31.set_ylabel(r"$[rad]$", rotation='vertical')
        ax31.legend([state_3_e, state_3_e_d],
                [r'$\psi$', r'$\psi_d$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax31.grid(color='#949494', linestyle='-.', linewidth=0.5)
        ax31.set_xlabel(r"$\textrm{Time}[s]$", labelpad=5)

        fig11.savefig(name + ".pdf")
        fig11.savefig(name + ".png")
        return None

def plot_time(fig11, ax11, x_sample, x_sample_real, t, name):
        t = t[0:x_sample.shape[1]]
        ax11.set_xlim((t[0], t[-1]))

        ax11.set_xticklabels([])
        state_1_e, = ax11.plot(t[0:t.shape[0]], x_sample[0, 0:t.shape[0]],
                    color='#C43C29', lw=1.0, ls="--")

        state_1_e_d, = ax11.plot(t[0:t.shape[0]], x_sample_real[0, 0:t.shape[0]],
                    color='#1D2121', lw=1.0, ls="-")


        ax11.set_ylabel(r"$[m]$", rotation='vertical')
        ax11.legend([state_1_e, state_1_e_d],
                [ r'$dt$', r'$sample~time$'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
        ax11.grid(color='#949494', linestyle='-.', linewidth=0.5)
        ax11.set_xlabel(r"$\textrm{Time}[s]$", labelpad=5)

        fig11.savefig(name + ".pdf")
        fig11.savefig(name + ".png")
        return None