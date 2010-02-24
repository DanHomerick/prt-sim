import xml.dom.minidom
import networkx
import sys

def main():
    print sys.argv[1]
    print sys.argv[2]
    graph = build_graph(sys.argv[1])
    output_shapes(graph)

def build_graph(xml_path):
    """Returns a networkx.DiGraph suitable for routing vehicles over."""
    graph = networkx.DiGraph()

    doc = xml.dom.minidom.parse(xml_path)
    track_segments_xml = doc.getElementsByTagName('TrackSegments')[0]

    for track_segment_xml in track_segments_xml.getElementsByTagName('TrackSegment'):
        start_xml = track_segment_xml.getElementsByTagName('Start')[0]
        end_xml = track_segment_xml.getElementsByTagName('End')[0]

        graph.add_edge( (float(start_xml.getAttribute('lat')), float(start_xml.getAttribute('lng'))),
                        (float(end_xml.getAttribute('lat')), float(end_xml.getAttribute('lng'))) )

    doc.unlink()
    return graph

def _to_numeric_id(element):
    """For elements similar to:
        <ID>x_trackSegment_forward</ID>
    where x is an integer. Returns just the integer value."""
    return int(element.childNodes[0].data.split('_')[0])

def output_shapes(graph):
    assert isinstance(graph, networkx.classes.DiGraph)
    out = open(sys.argv[2], 'w')
    out.write("shape_id,shape_pt_lat,shape_pt_lon,shape_pt_sequence\n")

    # *Assume* that we have a simple loop.
    start = node = graph.nodes()[0] # arbitrarily choose a node
    pt_seq = 1
    shape_id = 1
    while True:
        out.write("%d,%s,%d\n" % (shape_id, "%.5f,%.5f" % node, pt_seq))
        pt_seq += 1
        successors = graph.successors(node)
        assert len(successors) == 1
        node = successors[0]
        if node == start:
            break
    out.write("%d,%s,%d\n" % (shape_id, "%.5f,%.5f" % node, pt_seq))

##    paths = networkx.single_source_shortest_path(graph, graph.nodes()[0])
##    for shape_id, path in enumerate(paths.itervalues()):
##        for pt_seq, node in enumerate(path):
##            out.write("%d,%s,%d\n" % (shape_id+1, node, pt_seq+1))



if __name__ == '__main__':
    main()