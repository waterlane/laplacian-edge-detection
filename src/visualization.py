import plotly.graph_objects as go

def save_visualization_as_html(data, title, output_file):
    """
    Creates a visualization using Plotly and saves it as an HTML file.

    Parameters:
        data (list of dict): A list of dictionaries containing x, y, and name for each trace.
                             Example: [{'x': [1, 2, 3], 'y': [4, 5, 6], 'name': 'Trace 1'}, ...]
        title (str): The title of the plot.
        output_file (str): The path to save the HTML file.
    """
    fig = go.Figure()

    for trace in data:
        fig.add_trace(go.Scatter(x=trace['x'], y=trace['y'], mode='lines+markers', name=trace['name']))

    fig.update_layout(title=title, xaxis_title="X-axis", yaxis_title="Y-axis")

    fig.write_html(output_file)
    print(f"Visualization saved to {output_file}")