from sklearn.cluster import KMeans

class TPS():

    def __init__(self, verbose=0):
        self.verbose = verbose

    def segment_point_cloud(self, point_cloud, n_clusters=2, init="k-means++", n_init=20, max_iter=600, random_state=42, is_save=False, save_path=None):
        vertices = point_cloud.points

        kmeans = KMeans(
            n_clusters=n_clusters,      
            init=init,                 
            n_init=n_init,              
            max_iter=max_iter,            
            random_state=random_state   
        )

        labels = kmeans.fit_predict(vertices)

        color_map = {0: "part 0", 1: "part 1"}

        cluster_colors = [color_map[label] for label in labels]

        point_cloud["Labels"] = cluster_colors

        if(self.verbose > 0):
            point_cloud.plot(scalars="Labels", cmap="tab10")
        
        if(is_save):
            point_cloud.save(save_path)
            print("Point cloud saved to:", save_path)

    def segment_mesh(self, mesh, n_clusters=2, init="k-means++", n_init=10, max_iter=300, random_state=42, is_save=False, save_path=None):
        clean_mesh = mesh.clean()
        vertices = clean_mesh.points

        kmeans = KMeans(
            n_clusters=n_clusters,
            init=init,
            n_init=n_init,
            max_iter=max_iter,
            random_state=random_state
        )

        labels = kmeans.fit_predict(vertices)
        clean_mesh["Labels"] = labels

        if(self.verbose > 0):
            clean_mesh.plot(scalars="Labels")
        
        if(is_save):
            clean_mesh.save(save_path)
            print("Mesh saved to:", save_path)