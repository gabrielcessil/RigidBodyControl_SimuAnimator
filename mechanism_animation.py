import numpy as np
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.io as pio
import imageio
import tempfile
import os

class MechanicalSystemAnimation:
    def __init__(self, mechanism, system_name="Mechanical System"):
        if not hasattr(mechanism, 'time') or not hasattr(mechanism, 'states') or not hasattr(mechanism, 'direct_kinematics'):
            raise AttributeError("Mechanism must have 'time', 'states', and 'direct_kinematics'.")

        self.system_name = system_name
        self.time = mechanism.time
        self.states = mechanism.states
        self.state_names = list(self.states.keys())

        self.xyz_t = [
            mechanism.direct_kinematics([state_values[i] for state_values in self.states.values()])
            for i in range(len(self.time))
        ]

        self._calculate_3d_ranges()
        self._calculate_y_ranges()

        # Dynamic color assignment based on number of state_names
        num_elements = len(self.state_names)

        # Use Plotly qualitative color sets, repeat if needed to cover all elements
        base_colors = px.colors.qualitative.Set2
        self.colors_2d = (base_colors * ((num_elements // len(base_colors)) + 1))[:num_elements]

        base_colors_3d = px.colors.qualitative.Plotly
        self.colors_3d = (base_colors_3d * ((num_elements // len(base_colors_3d)) + 1))[:num_elements]

    def _calculate_3d_ranges(self):
        xs, ys, zs = zip(*[point for step in self.xyz_t for point in step])
        padding = 0.1
        self._x_range = [min(xs) - padding, max(xs) + padding]
        self._y_range = [min(ys) - padding, max(ys) + padding]
        self._z_range = [min(zs) - padding, max(zs) + padding]

    def _calculate_y_ranges(self):
        self.y_ranges = {}
        for name, values in self.states.items():
            ymin = min(values)
            ymax = max(values)
            padding = (ymax - ymin) * 0.1 if ymax != ymin else 1.0

            self.y_ranges[name] = [ymin - padding, ymax + padding]

            print(name,self.y_ranges[name] )

    def _compute_ticks(self, ymin, ymax, num_ticks=5):
        return np.linspace(ymin, ymax, num_ticks).round(3).tolist()

    def setup_figure(self, plots_per_page=4):
        num_states = len(self.state_names)
        rows = min(plots_per_page, num_states)

        specs = [[{"type": "scatter3d", "rowspan": rows}, {"type": "xy"}]] + [
            [None, {"type": "xy"}] for _ in range(rows - 1)
        ]

        # Row heights - distribute total height evenly among the rows
        row_heights = [1 / rows] * rows

        fig = make_subplots(
            rows=rows, cols=2,
            column_widths=[0.55, 0.45],
            specs=specs,
            horizontal_spacing=0.06,
            vertical_spacing=0.08,
            subplot_titles=[""] * (rows + 1),
            row_heights=row_heights
        )

        range_x = self._x_range[1] - self._x_range[0]
        range_y = self._y_range[1] - self._y_range[0]
        range_z = self._z_range[1] - self._z_range[0]
        max_range = max(range_x, range_y, range_z)

        camera = dict(
            center=dict(x=0, y=0, z=0),
            eye=dict(x=1.8, y=1.8, z=1.4),
            up=dict(x=0, y=0, z=1)
        )

        fig.update_layout(
            title=dict(
                text=f"<b>{self.system_name} - Kinematic Animation</b>",
                x=0.5,
                font=dict(size=18)
            ),
            font=dict(family="Times New Roman", size=12),
            margin=dict(l=20, r=140, t=70, b=30),
            paper_bgcolor="white",
            plot_bgcolor="white",
            hovermode="closest",
            showlegend=False,
        )

        fig.update_scenes(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Z (m)",
            aspectmode="manual",
            aspectratio=dict(
                x=range_x / max_range,
                y=range_y / max_range,
                z=range_z / max_range
            ),
            bgcolor="rgb(248,248,248)",
            xaxis=dict(
                showgrid=True, gridcolor="lightgray",
                zeroline=False, backgroundcolor="rgb(248,248,248)",
                range=self._x_range
            ),
            yaxis=dict(
                showgrid=True, gridcolor="lightgray",
                zeroline=False, backgroundcolor="rgb(248,248,248)",
                range=self._y_range
            ),
            zaxis=dict(
                showgrid=True, gridcolor="lightgray",
                zeroline=False, backgroundcolor="rgb(248,248,248)",
                range=self._z_range
            ),
            camera=camera
        )

        return fig

    def _create_3d_trace_at_time(self, i):
        positions = self.xyz_t[i]
        x, y, z = zip(*positions)

        return go.Scatter3d(
            x=x, y=y, z=z,
            mode='lines+markers',
            line=dict(color=self.colors_3d[0], width=4),
            marker=dict(size=6, color=self.colors_3d[1:]),
            hoverinfo='text',
            text=[f'Point {j + 1}: ({x[j]:.2f}, {y[j]:.2f}, {z[j]:.2f})' for j in range(len(x))],
            name="Mechanism",
            legendgroup="Mechanism",
            showlegend=False
        )

    def _create_2d_traces_at_time(self, i):
        traces = []
        for idx, name in enumerate(self.state_names):
            values = self.states[name]
            color = self.colors_2d[idx % len(self.colors_2d)]

            traces.append(
                go.Scatter(
                    x=self.time,
                    y=[v if j <= i else None for j, v in enumerate(values)],
                    mode="lines",
                    line=dict(color=color, width=2),
                    marker=dict(color=color),
                    hoverinfo='text',
                    text=[name],
                    name=name,
                    legendgroup=name,
                    showlegend=False,
                )
            )
        return traces

    def create_frames(self):
        frames = []
        for i in range(len(self.time)):
            frame_data = [self._create_3d_trace_at_time(i)]
            frame_data.extend(self._create_2d_traces_at_time(i))

            layout_updates = {}
            for idx, name in enumerate(self.state_names):
                row_in_page = (idx % 4) + 1
                y_range = self.y_ranges[name]

                layout_updates[f'yaxis{"" if row_in_page == 1 else row_in_page + 1}'] = dict(
                    range=y_range
                )

            frames.append(go.Frame(
                data=frame_data,
                name=str(i),
                layout=layout_updates
            ))

        return frames

    def create_animation(self, file_name="Mechanism_Animation"):
        num_states = len(self.state_names)
        plots_per_page = 4
        num_pages = (num_states + plots_per_page - 1) // plots_per_page
        rows = min(plots_per_page, num_states)

        fig = self.setup_figure()

        # Add 3D trace (first col spans all rows)
        fig.add_trace(self._create_3d_trace_at_time(0), row=1, col=1)

        # Add 2D traces with visibility toggled by page
        for idx, name in enumerate(self.state_names):
            row_in_page = (idx % plots_per_page) + 1
            page = idx // plots_per_page

            values = self.states[name]
            color = self.colors_2d[idx % len(self.colors_2d)]
            visible = (page == 0)  # Only first page visible initially
            fig.add_trace(
                go.Scatter(
                    x=self.time,
                    y=[values[0] if j == 0 else None for j in range(len(values))],
                    mode="lines",
                    line=dict(color=color, width=2),
                    marker=dict(color=color),
                    name=name,
                    visible=visible,
                    showlegend=False
                ),
                row=row_in_page, col=2
            )

            show_x = (row_in_page == plots_per_page) or (idx == num_states - 1)
            y_range = self.y_ranges[name]
            tickvals = self._compute_ticks(y_range[0], y_range[1], num_ticks=5)
            fig.update_xaxes(
                title_text="Time (s)" if show_x else "",
                showticklabels=show_x,
                range=[min(self.time), max(self.time)],
                row=row_in_page, col=2,
                showgrid=True, gridcolor="lightgray"
            )
            fig.update_yaxes(
                range=y_range,
                tickvals=tickvals,
                row=row_in_page, col=2,
                showgrid=True, gridcolor="lightgray",
                showticklabels=True
            )

        # Prepare annotations sets for each page
        annotations_by_page = []
        for page in range(num_pages):
            annots = []
            start_idx = page * plots_per_page
            end_idx = min(start_idx + plots_per_page, num_states)
            for idx in range(start_idx, end_idx):
                name = self.state_names[idx]
                color = self.colors_2d[idx % len(self.colors_2d)]
                y_pos = 0.95 - (idx - start_idx) * (1.1 / plots_per_page)
                annots.append(dict(
                    xref="paper", yref="paper",
                    x=0.99,
                    y=y_pos,
                    showarrow=False,
                    text=f"<span style='color:{color}'><b>■ {name}</b></span>",
                    font=dict(size=14),
                    align="left",
                    bgcolor="rgba(255,255,255,0.7)",
                    bordercolor="grey",
                    borderwidth=0.5,
                    yanchor='middle',
                ))

            annotations_by_page.append(annots)

        # *** FIX: Set initial annotations here so they appear on first load ***
        fig.update_layout(annotations=annotations_by_page[0])

        # Pagination buttons — positioned top-right above 2D plots
        pagination_buttons = []
        for page in range(num_pages):
            visibility = [True]  # 3D trace visible
            for idx in range(num_states):
                visibility.append((idx // plots_per_page) == page)

            annotations_for_page = annotations_by_page[page]

            pagination_buttons.append(dict(
                label=f"Page {page + 1}",
                method="update",
                args=[{"visible": visibility}, {"annotations": annotations_for_page}]
            ))

        # Animation Play/Pause buttons
        animation_buttons = [
            dict(label="▶ Play",
                 method="animate",
                 args=[None, {"frame": {"duration": 50, "redraw": True},
                              "fromcurrent": True, "mode": "immediate"}]),
            dict(label="■ Pause",
                 method="animate",
                 args=[[None], {"frame": {"duration": 0, "redraw": False},
                                "mode": "immediate"}])
        ]

        fig.update_layout(
            updatemenus=[
                dict(
                    type="buttons",
                    direction="right",
                    x=0.99,
                    y=1.02,
                    xanchor="right",
                    yanchor="bottom",
                    buttons=pagination_buttons,
                    pad={"r": 10, "t": 10},
                    showactive=True,
                    bgcolor="rgba(255,255,255,0.8)"
                ),
                dict(
                    type="buttons",
                    direction="right",
                    x=0.05, y=1.15,
                    xanchor="left",
                    yanchor="top",
                    buttons=animation_buttons,
                    pad={"r": 10, "t": 10},
                    showactive=True,
                    bgcolor="rgba(255,255,255,0.8)"
                )
            ]
        )

        frames = self.create_frames()
        fig.frames = frames

        fig.update_layout(
            sliders=[dict(
                steps=[dict(
                    args=[[f.name], {"frame": {"duration": 50, "redraw": True}, "mode": "immediate"}],
                    label=str(k),
                    method="animate"
                ) for k, f in enumerate(fig.frames)],
                transition=dict(duration=0),
                x=0.05,
                y=-0.05,
                len=0.9,
                currentvalue=dict(font=dict(size=12), prefix="Time: ", visible=True, xanchor="right"),
                bgcolor="lightgray",
                activebgcolor="darkgray",
                pad=dict(b=10, t=50)
            )]
        )

        try:
            pio.write_html(fig, file=file_name + ".html", auto_open=True,
                           full_html=True, include_plotlyjs='cdn')
            print(f"Animation saved as {file_name}.html and opened in your browser.")
        except Exception as e:
            print(f"Error saving or opening animation: {e}")

    def save_animation_as_gif(self, file_name="Mechanism_Animation", duration=50):
        """Render the animation frames and save as a GIF (duration in ms per frame)."""
        import imageio
        import tempfile
        import os

        frames = []
        temp_dir = tempfile.mkdtemp()
        try:
            print("Rendering frames to temporary PNG files...")
            for i in range(len(self.time)):
                fig = self.setup_figure()
                fig.add_trace(self._create_3d_trace_at_time(i), row=1, col=1)

                # Add 2D traces, showing data up to the current time step 'i'
                for idx, name in enumerate(self.state_names):
                    values = self.states[name]
                    color = self.colors_2d[idx % len(self.colors_2d)]
                    row_in_page = (idx % 4) + 1 # Calculate row for the subplot

                    fig.add_trace(
                        go.Scatter(
                            x=self.time[:i+1], # Only show data up to current time step
                            y=values[:i+1],   # Only show data up to current time step
                            mode="lines",
                            line=dict(color=color, width=2),
                            marker=dict(color=color),
                            name=name,
                            showlegend=False
                        ),
                        row=row_in_page, col=2
                    )

                    # Update axes for each 2D plot
                    show_x = (row_in_page == 4) or (idx == len(self.state_names) - 1)
                    y_range = self.y_ranges[name]
                    tickvals = self._compute_ticks(y_range[0], y_range[1], num_ticks=5)
                    fig.update_xaxes(
                        title_text="Time (s)" if show_x else "",
                        showticklabels=show_x,
                        range=[min(self.time), max(self.time)],
                        row=row_in_page, col=2,
                        showgrid=True, gridcolor="lightgray"
                    )
                    fig.update_yaxes(
                        range=self.y_ranges[name], # Corrected: use the pre-calculated range
                        tickvals=tickvals,
                        row=row_in_page, col=2,
                        showgrid=True, gridcolor="lightgray",
                        showticklabels=True
                    )

                file_path = os.path.join(temp_dir, f"frame_{i:04d}.png")
                fig.write_image(file_path, scale=2)
                frames.append(imageio.imread(file_path))
            gif_path = file_name + ".gif"
            imageio.mimsave(gif_path, frames, duration=duration / 1000)
            print(f"GIF saved as {gif_path}")
        finally:
            # Clean up temp files
            for f in os.listdir(temp_dir):
                os.remove(os.path.join(temp_dir, f))
            os.rmdir(temp_dir)